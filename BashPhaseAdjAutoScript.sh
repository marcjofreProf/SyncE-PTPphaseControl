#!/bin/bash
# Bash script to automatically execute the PI phase adjustment algorithm for SynE-PTP
# Usage: 
#   ./BashPhaseAdjAutoScript.sh             (Runs once, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 10          (Runs continuously, 15s interval, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 10 500      (Runs continuously, 15s interval, locks phase at +500 ps)

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
N=150                # Number of samples to collect per interval
TRIM_COUNT=4        # Trim average: Discard this many highest and lowest samples (e.g., 3 removes top 3 and bottom 3)
psCLK_OUTperiod=$(( 4000 * 9 ))      # Period of the CLK_OUT signal in picoseconds
psCLK_OUTperiodHalf=$((psCLK_OUTperiod/2))

# --- PI Controller Tuning ---
scaled_PID_factor=100      # Scaling value to operate with integers
scaled_PIDp=15             # Proportional gain (0.25)
scaled_PIDi=2              # Integral gain (0.02) - Keep this low!

# --- Initialize Persistent Memory ---
# Because the script stays alive, this variable simply persists in RAM!
INTEGRAL_ACCUM=0

if [ "$INTERVAL" -gt 0 ]; then
    echo "Starting SyncE-PTP PI Phase Controller (Continuous: ${INTERVAL}s interval | Target: ${TARGET_OFFSET}ps)..."
else
    echo "Starting SyncE-PTP PI Phase Controller (Single-shot mode | Target: ${TARGET_OFFSET}ps)..."
fi

# ==========================================
# --- Main Continuous Loop ---
# ==========================================
while true; do

    # Clear the terminal at the start of each new interval
    clear

    # --- Runtime Variables for this run ---
    UNWRAPPED_SAMPLES=()
    VALID_SAMPLES=0
    wrap_offset=0
    prev_val=""

    # Loop N times to collect samples
    for (( i=1; i<=$N; i++ ))
    do
        # 1. Clear the kernel ring buffer so we don't read stale data
        sudo dmesg -c > /dev/null
        
        sleep 0.20
        sudo phc_ctl $INTERFACE -- phaseadj 0 > /dev/null 2>&1
        sleep 0.05

        val=$(dmesg | grep "PHC_PHASE_RESULT:" | tail -1 | awk -F': ' '{print $NF}')
        
        # Strict check: Is it non-empty AND a valid integer?
        if ! [[ "$val" =~ ^-?[0-9]+$ ]]; then
            echo "Error: Invalid or missing offset from $INTERFACE at sample $i (Got: '$val')"
            continue
        fi

        # Sequential Phase Unwrapping
        if [ "$VALID_SAMPLES" -eq 0 ]; then
            prev_val=$val
            unwrapped_val=$val
        else
            diff=$(( val - prev_val ))
            if [ "$diff" -lt "-$psCLK_OUTperiodHalf" ]; then
                wrap_offset=$(( wrap_offset + psCLK_OUTperiod ))
            elif [ "$diff" -gt "$psCLK_OUTperiodHalf" ]; then
                wrap_offset=$(( wrap_offset - psCLK_OUTperiod ))
            fi
            prev_val=$val
            unwrapped_val=$(( val + wrap_offset ))
        fi

        # Store the unwrapped value in our array
        UNWRAPPED_SAMPLES+=("$unwrapped_val")
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
            for val in "${UNWRAPPED_SAMPLES[@]}"; do
                TRIMMED_SUM=$(( TRIMMED_SUM + val ))
            done
            RAW_AVERAGE=$(( TRIMMED_SUM / VALID_SAMPLES ))
        else
            # Sort the unwrapped array numerically
            SORTED=($(printf "%s\n" "${UNWRAPPED_SAMPLES[@]}" | sort -n))
            
            TRIMMED_SUM=0
            TRIMMED_COUNT=0
            
            # Sum the values, skipping the first $TRIM_COUNT and last $TRIM_COUNT items
            for (( j=TRIM_COUNT; j<VALID_SAMPLES-TRIM_COUNT; j++ ))
            do
                TRIMMED_SUM=$(( TRIMMED_SUM + SORTED[j] ))
                TRIMMED_COUNT=$(( TRIMMED_COUNT + 1 ))
            done
            
            RAW_AVERAGE=$(( TRIMMED_SUM / TRIMMED_COUNT ))
        fi

        # ==========================================
        # --- PI-Controller Math (Shortest Path) ---
        # ==========================================

        # 1. Calculate Signed Error (Inverted Logic: Process Variable - Setpoint)
        ERROR=$(( RAW_AVERAGE - TARGET_OFFSET ))

        # 2. Normalize Error to Shortest Path [-HalfPeriod, +HalfPeriod]
        # This fixes the circular wrap-around logic for the setpoint.
        ERROR=$(( ERROR % psCLK_OUTperiod ))
        if [ "$ERROR" -gt "$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( ERROR - psCLK_OUTperiod ))
        elif [ "$ERROR" -lt "-$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( ERROR + psCLK_OUTperiod ))
        fi

        # 3. Add Error to Integral Accumulator
        INTEGRAL_ACCUM=$(( INTEGRAL_ACCUM + ERROR ))

        # 4. Anti-Windup (Cap the integral accumulator at ± half period)
        if [ "$INTEGRAL_ACCUM" -gt "$psCLK_OUTperiodHalf" ]; then
            INTEGRAL_ACCUM=$psCLK_OUTperiodHalf
        elif [ "$INTEGRAL_ACCUM" -lt "-$psCLK_OUTperiodHalf" ]; then
            INTEGRAL_ACCUM=-$psCLK_OUTperiodHalf
        fi

        # 5. Calculate P and I terms
        P_TERM=$(( (scaled_PIDp * ERROR) / scaled_PID_factor ))
        I_TERM=$(( (scaled_PIDi * INTEGRAL_ACCUM) / scaled_PID_factor ))

        CORRECTIONscaled=$(( P_TERM + I_TERM ))

        # 6. Final Phase Output Wrap (Shortest Path logic for execution)
        # Prevents stepping by e.g. +80ns when -20ns achieves the exact same phase.
        CORRECTIONscaled=$(( CORRECTIONscaled % psCLK_OUTperiod ))
        if [ "$CORRECTIONscaled" -gt "$psCLK_OUTperiodHalf" ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled - psCLK_OUTperiod ))
        elif [ "$CORRECTIONscaled" -lt "-$psCLK_OUTperiodHalf" ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled + psCLK_OUTperiod ))
        fi

        # ==========================================
        # --- Execution ---
        # ==========================================

        ABS_CORRECTION=${CORRECTIONscaled#-}

        # Force a large correction
        # ABS_CORRECTION=$((ABS_CORRECTION+psCLK_OUTperiod))

        # Check if ptp4l is currently running. Somehow it is a bad behavior that when ptp4l running, the phase aligment procedure needs sort of the following (why 10? TODO)
        # Commented because done in the DP83640 driver
        #if pgrep "ptp4l" > /dev/null; then
        #    # If it is running, multiply the correction by 10
        #    ABS_CORRECTION=$((ABS_CORRECTION * 10))
        #    echo "ptp4l running, applying a multiplication factor"
        #fi

        if [[ "$CORRECTIONscaled" == -* ]]; then
            SIGN="-"
        else
            SIGN=""
        fi

        # Format picoseconds into fractional seconds
        CORRECTION=$(printf -- "%s0.%012d" "$SIGN" "$ABS_CORRECTION")

        if [ "$PlotInfo" = "true" ]; then
            echo "--- PID DIAGNOSTICS ---"
            if [ "$VALID_SAMPLES" -gt $(( TRIM_COUNT * 2 )) ]; then
                echo "Trimmed out $TRIM_COUNT high/low samples. Averaged middle $TRIMMED_COUNT."
            fi
            echo "Measured Raw Avg: $RAW_AVERAGE ps"
            if [ "$TARGET_OFFSET" -ne 0 ]; then
                echo "Target Offset: $TARGET_OFFSET ps"
            fi
            echo "Error: $ERROR ps | Accumulator: $INTEGRAL_ACCUM ps"
            echo "Applying P-Term: $P_TERM ps | I-Term: $I_TERM ps"
            echo "Total calculated step: $CORRECTIONscaled ps"
        fi

        sudo phc_ctl $INTERFACE -- phaseadj $CORRECTION
    fi

    # --- Loop Control ---
    if [ "$INTERVAL" -eq 0 ]; then
        break # Exit the loop immediately if running in single-shot mode
    fi

    if [ "$PlotInfo" = "true" ]; then
        echo "Sleeping for $INTERVAL seconds..."
        echo "-----------------------"
    fi
    sleep "$INTERVAL"

done