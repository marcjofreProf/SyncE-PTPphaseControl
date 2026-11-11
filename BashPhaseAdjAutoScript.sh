#!/bin/bash
# Bash script to automatically execute the PI phase adjustment algorithm for SynE-PTP
# Usage: 
#   ./BashPhaseAdjAutoScript.sh             (Runs continuously, 15s interval, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 30          (Runs continuously, 30s interval, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 15 500      (Runs continuously, 15s interval, locks phase at +500 ps)

clear

# Clean up the flag file from a previous run so PPS is re-aligned
# Make sure that execution rights are properly set: sudo chmod a+wrx ./BashPhaseAdjAutoScript.sh
rm -f /tmp/ptp_pps_active.flag

# --- CLI Arguments ---
INTERVAL=${1:-15}
if ! [[ "$INTERVAL" =~ ^[0-9]+$ ]]; then
    echo "Error: Interval argument must be a positive integer."
    exit 1
fi

TARGET_OFFSET=${2:-0}
if ! [[ "$TARGET_OFFSET" =~ ^-?[0-9]+$ ]]; then
    echo "Error: Target offset must be an integer."
    exit 1
fi

# --- Configuration ---
PlotInfo=true
INTERFACE="eth0"
N=45             # Number of samples to average
TRIM_COUNT=4

# CLK_OUT Period (40 ns = 40000 ps)
psCLK_OUTperiod=$(( 4000 * 10 ))
psCLK_OUTperiodHalf=$((psCLK_OUTperiod/2))

# AXI SyncE TX/RX Clock Period (25 MHz = 40 ns = 40000 ps)
AXI_PERIOD=$(( 4000 * 10 ))
AXI_PERIOD_HALF=$((AXI_PERIOD/2))

# --- FPGA Hardware Offset Configuration ---
AXI_PHASE_ADDR="0x43C00000"
AXI_PPS_TS_ADDR="0x43C00004"
AXI_CLKOUT_TS_ADDR="0x43C00008"
AXI_VALID_ADDR="0x43C0000C"

PS_PER_TICK=10000
PS_PER_TICK_factor=1
M3_FILTERED=0

# --- PI Controller Tuning ---
scaled_PID_factor=1000
scaled_PIDp=550
scaled_PIDi=50

INTEGRAL_ACCUM=0
BIAS_ACCUM=0        # Bias estimator in units of 1e-6 ps
BIAS_ESTIMATOR=0

# ==========================================
# --- PPS Activation Function ---
# ==========================================
ensure_pps_activated() {
    # Use a flag file in /tmp to remember if we already activated it.
    if [ -f /tmp/ptp_pps_active.flag ]; then
        return 0
    fi

    echo "PPS is not active. Enabling 1-PPS on GPIO3..."

    # 1. Disable any existing periodic output first
    sudo sh -c "echo '0 0 0 0 0' > /sys/class/ptp/ptp0/period" 2>/dev/null

    # 2. Check if GPIO3 is already configured as PEROUT
    if [ "$(cat /sys/class/ptp/ptp0/pins/GPIO3 2>/dev/null)" != "2 0" ]; then
        echo "Configuring GPIO3 as PEROUT..."
        sudo sh -c "echo '2 0' > /sys/class/ptp/ptp0/pins/GPIO3" || {
            echo "ERROR: Could not set GPIO3 to PEROUT."
            return 1
        }
    fi

    # 3. Get the current PHC time (seconds)
    PHC_TIME=$(sudo phc_ctl eth0 get | sed -n 's/.*clock time is \([0-9]*\)\..*/\1/p')
    if [ -z "$PHC_TIME" ]; then
        echo "ERROR: Could not read the PHC clock."
        return 1
    fi

    # 4. Round TARGET_OFFSET to nearest 8000 ps
    RESOLUTION_PS=8000
    if [ "$TARGET_OFFSET" -ge 0 ]; then
        ROUNDED_PS=$(( (TARGET_OFFSET + RESOLUTION_PS/2) / RESOLUTION_PS * RESOLUTION_PS ))
    else
        ROUNDED_PS=$(( (TARGET_OFFSET - RESOLUTION_PS/2) / RESOLUTION_PS * RESOLUTION_PS ))
    fi

    ROUNDED_NS=$(( ROUNDED_PS / 1000 ))

    # 5. Calculate the NEXT ABSOLUTE 10-SECOND BOUNDARY + offset
    START_TIME=$(( ((PHC_TIME + 10) / 10) * 10 ))
    START_NSEC=$(( ROUNDED_NS ))
    
    while [ "$START_NSEC" -lt 0 ]; do
        START_NSEC=$(( START_NSEC + 1000000000 ))
        START_TIME=$(( START_TIME - 1 ))
    done
    if [ "$START_NSEC" -ge 1000000000 ]; then
        START_NSEC=$(( START_NSEC - 1000000000 ))
        START_TIME=$(( START_TIME + 1 ))
    fi

    # 6. Write the period
    if ! sudo sh -c "echo '0 $START_TIME $START_NSEC 1 0' > /sys/class/ptp/ptp0/period"; then
        echo "ERROR: Failed to enable PPS."
        return 1
    fi

    # Create the flag file so we don't activate it again
    touch /tmp/ptp_pps_active.flag

    echo "1-PPS enabled. First edge aligned to ABSOLUTE time: ${START_TIME}.${START_NSEC}."
    return 0
}

# Initial activation
ensure_pps_activated

echo "Starting SyncE-PTP PI Phase Controller (Continuous: ${INTERVAL}s interval | Target: ${TARGET_OFFSET}ps)..."

# ==========================================
# --- Main Continuous Loop ---
# ==========================================
while true; do
    ensure_pps_activated

    TICK_SAMPLES=()
    M3_SAMPLES=()
    AXI_OFFSET_SAMPLES=()
    LOOP_COUNT=0

    for (( i=1; i<=$N; i++ ))
    do
        sudo dmesg -c > /dev/null
        sleep 2.1      # It has to be larger than 1s to account for different measurements with respect the PPS signal

        AXI_RAW_HEX=$(sudo devmem $AXI_PHASE_ADDR 32 2>/dev/null)
        PPS_HEX=$(sudo devmem $AXI_PPS_TS_ADDR 32 2>/dev/null)
        CLK_HEX=$(sudo devmem $AXI_CLKOUT_TS_ADDR 32 2>/dev/null)
        VALID_HEX=$(sudo devmem $AXI_VALID_ADDR 32 2>/dev/null)
        
        VALID_CLEAN=$(echo "$VALID_HEX" | sed 's/0x//; s/^0*//')
        
        if [[ "$VALID_CLEAN" == "3" ]]; then
            PPS_DEC=$((16#${PPS_HEX#0x}))
            CLK_DEC=$((16#${CLK_HEX#0x}))
            
            M3_RAW_TICKS=$(( CLK_DEC - PPS_DEC ))
            if [ "$M3_RAW_TICKS" -lt 0 ]; then
                M3_RAW_TICKS=$(( M3_RAW_TICKS + 4294967296 ))
            fi
            
            M3_RAW_PS=$(( M3_RAW_TICKS * PS_PER_TICK / PS_PER_TICK_factor ))
            M3_RAW_PS=$(( M3_RAW_PS % psCLK_OUTperiod ))
            if [ "$M3_RAW_PS" -ge "$psCLK_OUTperiodHalf" ]; then
                M3_RAW_PS=$(( M3_RAW_PS - psCLK_OUTperiod ))
            fi
            M3_SAMPLES+=("$M3_RAW_PS")
        fi

        if [[ "$VALID_CLEAN" == "3" ]] && [[ "$AXI_RAW_HEX" =~ ^0x[0-9a-fA-F]+$ ]]; then
            unsigned_val=$(printf "%d" "$AXI_RAW_HEX")
            if [ "$unsigned_val" -ge 2147483648 ]; then
                curr_tick=$(( unsigned_val - 4294967296 ))
            else
                curr_tick=$unsigned_val
            fi

            # Convert ticks to picoseconds
            curr_axi_ps=$(( curr_tick * 2081 / 100 ))
            
            # Wrap AXI to the DP83640 8ns resolution window [-4000, 4000]
            curr_axi_ps=$(( curr_axi_ps % AXI_PERIOD ))
            if [ "$curr_axi_ps" -ge "$AXI_PERIOD_HALF" ]; then
                curr_axi_ps=$(( curr_axi_ps - AXI_PERIOD ))
            fi
            
            AXI_OFFSET_SAMPLES+=("$curr_axi_ps")
            TICK_SAMPLES+=("$curr_tick")
        fi

        LOOP_COUNT=$(( LOOP_COUNT + 1 ))
    done

    if [ "$LOOP_COUNT" -eq 0 ]; then
        echo "Error: No samples collected."
    else
        if [ ${#TICK_SAMPLES[@]} -eq 0 ]; then
            AXI_TICKS=0
        elif [ ${#TICK_SAMPLES[@]} -le $(( TRIM_COUNT * 2 )) ]; then
            TRIMMED_TICK_SUM=0
            for t_val in "${TICK_SAMPLES[@]}"; do
                TRIMMED_TICK_SUM=$(( TRIMMED_TICK_SUM + t_val ))
            done
            AXI_TICKS=$(( TRIMMED_TICK_SUM / ${#TICK_SAMPLES[@]} ))
        else
            SORTED_TICKS=($(printf "%s\n" "${TICK_SAMPLES[@]}" | sort -n))
            TRIMMED_TICK_SUM=0
            TRIMMED_COUNT=0
            for (( j=TRIM_COUNT; j<${#SORTED_TICKS[@]}-TRIM_COUNT; j++ ))
            do
                TRIMMED_TICK_SUM=$(( TRIMMED_TICK_SUM + SORTED_TICKS[j] ))
                TRIMMED_COUNT=$(( TRIMMED_COUNT + 1 ))
            done
            AXI_TICKS=$(( TRIMMED_TICK_SUM / TRIMMED_COUNT ))
        fi

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

        # Error = (Physical Phase from FPGA) - (Hardware SyncE Phase)
        M3_FILTERED=$M3_AVG
        # For a Master board the RAW_ERROR is: RAW_ERROR=$(( M3_FILTERED - AXI_AVG ))
        RAW_ERROR=$(( M3_FILTERED + AXI_AVG ))

        # Wrap the raw error to the shortest path [-20000, 20000]
        RAW_ERROR=$(( RAW_ERROR % psCLK_OUTperiod ))
        if [ "$RAW_ERROR" -lt "-$psCLK_OUTperiodHalf" ]; then
            RAW_ERROR=$(( RAW_ERROR + psCLK_OUTperiod ))
        elif [ "$RAW_ERROR" -ge "$psCLK_OUTperiodHalf" ]; then
            RAW_ERROR=$(( RAW_ERROR - psCLK_OUTperiod ))
        fi

        # --- Bias Estimator ---
        # Slowly learn the constant hardware offset (AXI bias)
        # We use a very slow filter: 1/64th of the new error.
        # This prevents the bias from adapting to fast noise, but it will track the constant offset.
        # Bias Estimator (IIR filter with 1/64 gain, scaled by 1000 to avoid truncation)
        BIAS_ACCUM=$(( BIAS_ACCUM + (RAW_ERROR * 1000 - BIAS_ACCUM) / 64 ))
        BIAS_ESTIMATOR=$(( BIAS_ACCUM / 1000 ))

        # The actual error fed to the PI controller is the raw error minus the learned bias
        ERROR=$(( RAW_ERROR - BIAS_ESTIMATOR ))

        # Wrap the corrected error to the shortest path [-20000, 20000]
        ERROR=$(( ERROR % psCLK_OUTperiod ))
        if [ "$ERROR" -lt "-$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( ERROR + psCLK_OUTperiod ))
        elif [ "$ERROR" -ge "$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( ERROR - psCLK_OUTperiod ))
        fi

        INTEGRAL_ACCUM=$(( INTEGRAL_ACCUM + ERROR ))
        if [ "$INTEGRAL_ACCUM" -gt "$psCLK_OUTperiodHalf" ]; then
            INTEGRAL_ACCUM=$psCLK_OUTperiodHalf
        elif [ "$INTEGRAL_ACCUM" -lt "-$psCLK_OUTperiodHalf" ]; then
            INTEGRAL_ACCUM=-$psCLK_OUTperiodHalf
        fi

        P_TERM=$(( (scaled_PIDp * ERROR) / scaled_PID_factor ))
        I_TERM=$(( (scaled_PIDi * INTEGRAL_ACCUM) / scaled_PID_factor ))
        CORRECTIONscaled=$(( P_TERM + I_TERM ))

        CORRECTIONscaled=$(( CORRECTIONscaled % psCLK_OUTperiod ))
        if [ "$CORRECTIONscaled" -lt "-$psCLK_OUTperiodHalf" ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled + psCLK_OUTperiod ))
        elif [ "$CORRECTIONscaled" -ge "$psCLK_OUTperiodHalf" ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled - psCLK_OUTperiod ))
        fi

        ABS_CORRECTION=${CORRECTIONscaled#-}
        if [[ "$CORRECTIONscaled" == -* ]]; then
            SIGN="-"
        else
            SIGN=""
        fi
        CORRECTION=$(printf -- "%s0.%012d" "$SIGN" "$ABS_CORRECTION")

        if [ "$PlotInfo" = "true" ]; then
            clear
            echo "--- PID DIAGNOSTICS ($(date +%T)) ---"
            echo "PPS vs CLK_OUT (M3):     $M3_AVG ps"
            echo "SyncE TX/RX (Diag):      $AXI_TICKS ticks -> $AXI_AVG ps"
            echo "Raw Error (M3 + AXI):    $RAW_ERROR ps"
            echo "Learned Bias:            $BIAS_ESTIMATOR ps"
            echo "Corrected Error:         $ERROR ps"
            echo "Accumulator:             $INTEGRAL_ACCUM ps"
            echo "Total calculated step: $CORRECTIONscaled ps"
            echo "Sleeping for $INTERVAL seconds..."
        fi

        if [ "$CORRECTIONscaled" -ne 0 ]; then
            sudo phc_ctl $INTERFACE -- phaseadj $CORRECTION
        fi
    fi

    if [ "$INTERVAL" -eq 0 ]; then
        break
    fi

    sleep "$INTERVAL"
done
