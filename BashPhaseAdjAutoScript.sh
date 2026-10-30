#!/bin/bash
# Bash script to automatically execute the PI phase adjustment algorithm for SynE-PTP
# Usage: 
#   ./BashPhaseAdjAutoScript.sh             (Runs once, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 15          (Runs continuously, 15s interval, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 15 500      (Runs continuously, 15s interval, locks phase at +500 ps)

# Note: Still, since the internal DP83640 PHY PTP clock is actually of 8 ns, everynow and then it can fall one cycle (or half cycle) out of the true center.
#       This has to do with each specify PHY internal compensation in dp83640.c driver: correction = (((3 * reference_clock_period) + 11) * 1000).
#       Hence, a sort of calibration or manual compensation could be required.
#       Or, the CLK_OUT signal should be introduced in IP block to measure this offset.

# Silence kernel console output (level 5 = KERN_NOTICE).
# The messages are still saved in the ring buffer for dmesg.
sudo dmesg -n 4
clear
# --- CLI Arguments ---
# 1. Interval Argument
INTERVAL=${1:-0}
if ! [[ "$INTERVAL" =~ ^[0-9]+$ ]]; then
    echo "Error: Interval argument must be a positive integer (e.g., 15)."
    exit 1
fi

# 2. Target Offset Argument (Systematic Phase Setpoint in ps)
TARGET_OFFSET=${2:-0}
if ! [[ "$TARGET_OFFSET" =~ ^-?[0-9]+$ ]]; then
    echo "Error: Target offset must be an integer (e.g., -1000 or 500)."
    exit 1
fi

# --- Configuration ---
PlotInfo=true       # Set to true to see the PID math
INTERFACE="eth0"
N=25                 # Number of samples to collect per interval
TRIM_COUNT=4         # Trim average: Discard this many highest and lowest samples
psCLK_OUTperiod=$(( 4000 * 10 ))      # Period of the CLK_OUT signal in picoseconds
psCLK_OUTperiodHalf=$((psCLK_OUTperiod/2))

# --- FPGA Hardware Offset Configuration ---
AXI_PHASE_ADDR="0x43C00000"  # Base address
AXI_PPS_TS_ADDR="0x43C00004" # PPS Timestamp
AXI_CLKOUT_TS_ADDR="0x43C00008" # CLKOUT Timestamp
AXI_VALID_ADDR="0x43C0000C"  # Validity flags

PS_PER_TICK=10000            # 1 / 100 MHz = 10 ns = 10000 ps
PS_PER_TICK_factor=1         # We don't need scaling anymore since the math fits in 64-bit integers
M3_FILTER_N=10               # Slow filter for the PPS/CLK_OUT difference
M3_FILTERED=0

# --- PI Controller Tuning ---
scaled_PID_factor=1000      # Scaling value to operate with integers
scaled_PIDp=750             # Proportional gain (0.700)
scaled_PIDi=50              # Integral gain (0.250)

# --- Initialize Persistent Memory ---
INTEGRAL_ACCUM=0

if [ "$INTERVAL" -gt 0 ]; then
    echo "Starting SyncE-PTP PI Phase Controller (Continuous: ${INTERVAL}s interval | Target: ${TARGET_OFFSET}ps)..."
else
    echo "Starting SyncE-PTP PI Phase Controller (Single-shot mode | Target: ${TARGET_OFFSET}ps)..."
fi

# ==========================================
# --- Activate the PPS Signal on GPIO3 ---
# ==========================================
echo "Enabling 1-PPS on GPIO3..."

# 1. Disable any existing periodic output first
sudo sh -c "echo '0 0 0 0 0' > /sys/class/ptp/ptp0/period" 2>/dev/null

# 2. Check if GPIO3 is already configured as PEROUT (function 2, channel 0)
if [ "$(cat /sys/class/ptp/ptp0/pins/GPIO3)" != "2 0" ]; then
    echo "Configuring GPIO3 as PEROUT..."
    if ! sudo sh -c "echo '2 0' > /sys/class/ptp/ptp0/pins/GPIO3"; then
        echo "ERROR: Could not set GPIO3 to PEROUT. Check that the driver is patched correctly."
        exit 1
    fi
fi

# 3. Get the current PHC time
PHC_TIME=$(sudo phc_ctl eth0 get | sed -n 's/.*clock time is \([0-9]*\)\..*/\1/p')
if [ -z "$PHC_TIME" ]; then
    echo "ERROR: Could not read the PHC clock. Is the link up?"
    exit 1
fi

# 4. Write the period with a start time 3 seconds in the future.
START_TIME=$((PHC_TIME + 3))
if ! sudo sh -c "echo '0 $START_TIME 0 1 0' > /sys/class/ptp/ptp0/period"; then
    echo "ERROR: Failed to enable PPS. Is the Ethernet cable plugged in and the link UP?"
    exit 1
fi
echo "1-PPS enabled. First edge at PHC time $START_TIME."

# ==========================================
# --- Main Continuous Loop ---
# ==========================================
while true; do
    # --- Runtime Variables for this run ---
    UNWRAPPED_SAMPLES=()
    TICK_SAMPLES=()
    M3_SAMPLES=()
    AXI_OFFSET_SAMPLES=()
    VALID_SAMPLES=0

    # Loop N times to collect samples
    for (( i=1; i<=$N; i++ ))
    do
        # 1. Clear the kernel ring buffer so we don't read stale data
        sudo dmesg -c > /dev/null
        
        sleep 0.80
        sudo phc_ctl $INTERFACE -- phaseadj 0 > /dev/null 2>&1
        sleep 0.20

        val=$(dmesg | grep "PHC_PHASE_CLKOUT_PS:" | tail -1 | awk -F': ' '{print $NF}')
        
        # Strict check: Is it non-empty AND a valid integer?
        if ! [[ "$val" =~ ^-?[0-9]+$ ]]; then
            echo "Error: Invalid or missing offset from $INTERFACE at sample $i (Got: '$val')"
            continue
        fi

        val=$(( val % psCLK_OUTperiod ))
        if [ "$val" -gt "$psCLK_OUTperiodHalf" ]; then
            val=$(( val - psCLK_OUTperiod ))
        elif [ "$val" -le -$psCLK_OUTperiodHalf ]; then
            val=$(( val + psCLK_OUTperiod ))
        fi

        # Store the unwrapped value in our array
        UNWRAPPED_SAMPLES+=("$val")

        # --- Read Hardware Phase from FPGA during sample collection ---
        AXI_RAW_HEX=$(sudo devmem $AXI_PHASE_ADDR 32 2>/dev/null)
        
        # --- Read PPS & CLKOUT Timestamps ---
        PPS_HEX=$(sudo devmem $AXI_PPS_TS_ADDR 32 2>/dev/null)
        CLK_HEX=$(sudo devmem $AXI_CLKOUT_TS_ADDR 32 2>/dev/null)
        VALID_HEX=$(sudo devmem $AXI_VALID_ADDR 32 2>/dev/null)
        
        # Strip the "0x" and leading zeros to ensure the comparison is safe
        VALID_CLEAN=$(echo "$VALID_HEX" | sed 's/0x//; s/^0*//')
        
        if [[ "$VALID_CLEAN" == "3" ]]; then
            # Force Bash to read the hex as an unsigned 32-bit integer
            PPS_DEC=$((16#${PPS_HEX#0x}))
            CLK_DEC=$((16#${CLK_HEX#0x}))
            
            M3_RAW_TICKS=$(( CLK_DEC - PPS_DEC ))
            
            # Handle wrap-around for the 32-bit counter
            if [ "$M3_RAW_TICKS" -lt 0 ]; then
                M3_RAW_TICKS=$(( M3_RAW_TICKS + 4294967296 ))
            fi
            
            M3_RAW_PS=$(( M3_RAW_TICKS * PS_PER_TICK / PS_PER_TICK_factor ))
            
            # The DP83640 has an 8 ns internal resolution. The PPS and CLK_OUT 
            # are generated by dividing the 125 MHz reference (8 ns period).
            # Therefore, we wrap M3 to the 8 ns period, not 40 ns.
            M3_RAW_PS=$(( M3_RAW_PS % 8000 ))
            
            # If > 4 ns, treat as negative phase
            if [ "$M3_RAW_PS" -gt 4000 ]; then
                M3_RAW_PS=$(( M3_RAW_PS - 8000 ))
            fi
            
            # Add to M3 array for averaging
            M3_SAMPLES+=("$M3_RAW_PS")
        fi

        if [[ "$AXI_RAW_HEX" =~ ^0x[0-9a-fA-F]+$ ]]; then
            # 1. Convert hex to unsigned decimal
            unsigned_val=$(printf "%d" "$AXI_RAW_HEX")
            
            # 2. Convert unsigned 32-bit to signed 32-bit
            if [ "$unsigned_val" -ge 2147483648 ]; then
                curr_tick=$(( unsigned_val - 4294967296 ))
            else
                curr_tick=$unsigned_val
            fi
        else
            curr_tick=0
        fi

        # Calculate the AXI offset for this single sample (in ps)
        curr_axi_ps=$(( curr_tick * 2081 / 100 ))
        curr_axi_ps=$(( ( curr_axi_ps % 8000 ) - 4000 ))
        AXI_OFFSET_SAMPLES+=("$curr_axi_ps")

        TICK_SAMPLES+=("$curr_tick")

        VALID_SAMPLES=$(( VALID_SAMPLES + 1 ))
    done

    # Protect against total read failure
    if [ "$VALID_SAMPLES" -eq 0 ]; then
        echo "Error: No valid samples collected. Skipping this interval."
    else
        # ==========================================
        # --- Trimmed Averaging Math ---
        # ==========================================
        if [ "$VALID_SAMPLES" -le $(( TRIM_COUNT * 2 )) ]; then
            # Not enough samples to trim safely, fallback to a normal average
            echo "Warning: Not enough samples for trimmed average. Doing normal mean."
            TRIMMED_SUM=0
            TRIMMED_TICK_SUM=0
            for val in "${UNWRAPPED_SAMPLES[@]}"; do
                TRIMMED_SUM=$(( TRIMMED_SUM + val ))
            done
            for t_val in "${TICK_SAMPLES[@]}"; do
                TRIMMED_TICK_SUM=$(( TRIMMED_TICK_SUM + t_val ))
            done
            RAW_AVERAGE=$(( TRIMMED_SUM / VALID_SAMPLES ))
            AXI_TICKS=$(( TRIMMED_TICK_SUM / VALID_SAMPLES ))
        else
            # Sort the unwrapped array and tick array numerically
            SORTED=($(printf "%s\n" "${UNWRAPPED_SAMPLES[@]}" | sort -n))
            SORTED_TICKS=($(printf "%s\n" "${TICK_SAMPLES[@]}" | sort -n))
            
            TRIMMED_SUM=0
            TRIMMED_TICK_SUM=0
            TRIMMED_COUNT=0
            
            for (( j=TRIM_COUNT; j<VALID_SAMPLES-TRIM_COUNT; j++ ))
            do
                TRIMMED_SUM=$(( TRIMMED_SUM + SORTED[j] ))
                TRIMMED_TICK_SUM=$(( TRIMMED_TICK_SUM + SORTED_TICKS[j] ))
                TRIMMED_COUNT=$(( TRIMMED_COUNT + 1 ))
            done
            
            RAW_AVERAGE=$(( TRIMMED_SUM / TRIMMED_COUNT ))
            AXI_TICKS=$(( TRIMMED_TICK_SUM / TRIMMED_COUNT ))
        fi

        # ==========================================
        # --- Average M3 and AXI over the interval ---
        # ==========================================
        M3_AVG=0
        COUNT_M3=0
        if [ ${#M3_SAMPLES[@]} -gt 0 ]; then
            SORTED_M3=($(printf "%s\n" "${M3_SAMPLES[@]}" | sort -n))
            if [ ${#SORTED_M3[@]} -le $(( TRIM_COUNT * 2 )) ]; then
                for val in "${SORTED_M3[@]}"; do
                    M3_AVG=$(( M3_AVG + val ))
                    COUNT_M3=$(( COUNT_M3 + 1 ))
                done
            else
                for (( j=TRIM_COUNT; j<${#SORTED_M3[@]}-TRIM_COUNT; j++ ))
                do
                    M3_AVG=$(( M3_AVG + SORTED_M3[j] ))
                    COUNT_M3=$(( COUNT_M3 + 1 ))
                done
            fi
            if [ "$COUNT_M3" -gt 0 ]; then
                M3_AVG=$(( M3_AVG / COUNT_M3 ))
            fi
        fi

        AXI_AVG=0
        COUNT_AXI=0
        if [ ${#AXI_OFFSET_SAMPLES[@]} -gt 0 ]; then
            SORTED_AXI=($(printf "%s\n" "${AXI_OFFSET_SAMPLES[@]}" | sort -n))
            if [ ${#SORTED_AXI[@]} -le $(( TRIM_COUNT * 2 )) ]; then
                for val in "${SORTED_AXI[@]}"; do
                    AXI_AVG=$(( AXI_AVG + val ))
                    COUNT_AXI=$(( COUNT_AXI + 1 ))
                done
            else
                for (( j=TRIM_COUNT; j<${#SORTED_AXI[@]}-TRIM_COUNT; j++ ))
                do
                    AXI_AVG=$(( AXI_AVG + SORTED_AXI[j] ))
                    COUNT_AXI=$(( COUNT_AXI + 1 ))
                done
            fi
            if [ "$COUNT_AXI" -gt 0 ]; then
                AXI_AVG=$(( AXI_AVG / COUNT_AXI ))
            fi
        fi

        # ==========================================
        # --- LOOP III: Average PPS vs CLK_OUT (M3) ---
        # ==========================================
        if [ "$M3_AVG" -ne 0 ]; then
            M3_FILTERED=$(( (M3_FILTERED * (M3_FILTER_N - 1) + M3_AVG) / M3_FILTER_N ))
        fi

        # ==========================================
        # --- LOOP I: Error Calculation ---
        # ==========================================
        COMPENSATED_PHASE=$(( RAW_AVERAGE ))
        ERROR=$(( COMPENSATED_PHASE - (TARGET_OFFSET + M3_FILTERED) ))

        # ==========================================
        # --- PI-Controller Math (Shortest Path) ---
        # ==========================================

        # 1. Normalize Error to Shortest Path [-HalfPeriod, +HalfPeriod]
        ERROR=$(( ERROR % psCLK_OUTperiod ))
        if [ "$ERROR" -le -$psCLK_OUTperiodHalf ]; then
            ERROR=$(( ERROR + psCLK_OUTperiod ))
        elif [ "$ERROR" -gt "$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( ERROR - psCLK_OUTperiod ))
        fi

        # 2. Add Error to Integral Accumulator
        INTEGRAL_ACCUM=$(( INTEGRAL_ACCUM + ERROR ))

        # 3. Anti-Windup (Cap the integral accumulator at ± half period)
        if [ "$INTEGRAL_ACCUM" -gt "$psCLK_OUTperiodHalf" ]; then
            INTEGRAL_ACCUM=$psCLK_OUTperiodHalf
        elif [ "$INTEGRAL_ACCUM" -lt "-$psCLK_OUTperiodHalf" ]; then
            INTEGRAL_ACCUM=-$psCLK_OUTperiodHalf
        fi

        # 4. Calculate P and I terms
        P_TERM=$(( (scaled_PIDp * ERROR) / scaled_PID_factor ))
        I_TERM=$(( (scaled_PIDi * INTEGRAL_ACCUM) / scaled_PID_factor ))

        CORRECTIONscaled=$(( P_TERM + I_TERM ))

        # 5. Final Phase Output Wrap (Shortest Path logic for execution)
        CORRECTIONscaled=$(( CORRECTIONscaled % psCLK_OUTperiod ))
        if [ "$CORRECTIONscaled" -le -$psCLK_OUTperiodHalf ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled + psCLK_OUTperiod ))
        elif [ "$CORRECTIONscaled" -gt "$psCLK_OUTperiodHalf" ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled - psCLK_OUTperiod ))
        fi

        # ==========================================
        # --- Execution ---
        # ==========================================

        ABS_CORRECTION=${CORRECTIONscaled#-}

        if [[ "$CORRECTIONscaled" == -* ]]; then
            SIGN=""
        else
            SIGN="-"
        fi

        # Format picoseconds into fractional seconds
        CORRECTION=$(printf -- "%s0.%012d" "$SIGN" "$ABS_CORRECTION")

        if [ "$PlotInfo" = "true" ]; then
            # NOW clear the screen (this is the only place we clear),
            # so the old summary disappears right before the new one appears.
            clear

            echo "--- PID DIAGNOSTICS ($(date +%T)) ---"
            if [ "$VALID_SAMPLES" -gt $(( TRIM_COUNT * 2 )) ]; then
                echo "Trimmed out $TRIM_COUNT high/low samples. Averaged middle $TRIMMED_COUNT."
            fi
            echo "CLK_OUT Phase (DP83640): $RAW_AVERAGE ps"
            echo "PPS vs CLK_OUT (M3):     $M3_AVG ps (Filtered: $M3_FILTERED ps)"
            echo "SyncE TX/RX (Diag):      $AXI_TICKS ticks -> $AXI_AVG ps"

            if [ "$TARGET_OFFSET" -ne 0 ]; then
                echo "Target Setpoint Offset:  $TARGET_OFFSET ps"
            fi

            echo "-----------------------"
            echo "Error: $ERROR ps | Accumulator: $INTEGRAL_ACCUM ps"
            echo "Applying P-Term: $P_TERM ps | I-Term: $I_TERM ps"
            echo "Total calculated step: $CORRECTIONscaled ps"
            echo "======================="
            echo "Sleeping for $INTERVAL seconds..."
        fi

        sudo phc_ctl $INTERFACE -- phaseadj $CORRECTION
    fi

    # --- Loop Control ---
    if [ "$INTERVAL" -eq 0 ]; then
        break # Exit the loop immediately if running in single-shot mode
    fi

    sleep "$INTERVAL"

done