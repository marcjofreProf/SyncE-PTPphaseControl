#!/bin/bash
# Bash script to automatically execute the PI phase adjustment algorithm for SynE-PTP
# Usage: 
#   ./BashPhaseAdjAutoScript.sh             (Runs once, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 15          (Runs continuously, 15s interval, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 15 500      (Runs continuously, 15s interval, locks phase at +500 ps)

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
N=15                 # Lowered to 5 to reduce "dead time" delay
psCLK_OUTperiod=100000      # Period of the CLK_OUT signal in picoseconds
psCLK_OUTperiodHalf=$((psCLK_OUTperiod/2))

# --- PI Controller Tuning ---
scaled_PID_factor=100      # Scaling value to operate with integers
scaled_PIDp=80             # Proportional gain (0.40)
scaled_PIDi=5              # Integral gain (0.05) - Keep this low!

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

    # --- Runtime Variables for this run ---
    SUM=0
    VALID_SAMPLES=0
    wrap_offset=0
    prev_val=""

    # Loop N times to collect samples
    for (( i=1; i<=$N; i++ ))
    do
        sleep 1.9 
        sudo phc_ctl $INTERFACE -- phaseadj 0 > /dev/null 2>&1
        sleep 0.1

        val=$(dmesg | grep "PHC_PHASE_RESULT:" | tail -1 | awk -F': ' '{print $NF}')
        
        if [[ -z "$val" ]]; then
            echo "Error: Could not read offset from $INTERFACE at sample $i"
            continue
        fi

        # Sequential Unwrapping
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

        SUM=$(( SUM + unwrapped_val ))
        VALID_SAMPLES=$(( VALID_SAMPLES + 1 ))
    done

    # Protect against total read failure
    if [ "$VALID_SAMPLES" -eq 0 ]; then
        echo "Error: No valid samples collected. Skipping this interval."
    else
        RAW_AVERAGE=$(( SUM / VALID_SAMPLES ))
        
        # Apply the systematic phase target offset!
        AVERAGE=$(( RAW_AVERAGE - TARGET_OFFSET ))
        
        # Keep the shifted average mathematically bounded [0, fullperiod]
        AVERAGE=$(( AVERAGE % psCLK_OUTperiod ))
        if [ "$AVERAGE" -lt 0 ]; then
            AVERAGE=$(( AVERAGE + psCLK_OUTperiod ))
        fi

        # ==========================================
        # --- PI-Controller Math ---
        # ==========================================

        # 1. Calculate the Signed Error (Hardware-Matched Polarity)
        if [ "$AVERAGE" -gt "$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( AVERAGE - psCLK_OUTperiod ))
        else
            ERROR=$AVERAGE
        fi

        # 2. Add Error to Integral Accumulator (Persists across loop iterations)
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

        # 5. Boundary Protection
        if [ "$CORRECTIONscaled" -ge "$psCLK_OUTperiod" ]; then
            CORRECTIONscaled=$((CORRECTIONscaled - psCLK_OUTperiod))
        elif [ "$CORRECTIONscaled" -le "-$psCLK_OUTperiod" ]; then
            CORRECTIONscaled=$((CORRECTIONscaled + psCLK_OUTperiod))
        fi

        # ==========================================
        # --- Execution ---
        # ==========================================

        ABS_CORRECTION=${CORRECTIONscaled#-}

        if [[ "$CORRECTIONscaled" == -* ]]; then
            SIGN="-"
        else
            SIGN=""
        fi

        CORRECTION=$(printf -- "%s0.%012d" "$SIGN" "$ABS_CORRECTION")

        if [ "$PlotInfo" = "true" ]; then
            echo "--- PID DIAGNOSTICS ---"
            echo "Measured Raw Avg: $RAW_AVERAGE ps"
            if [ "$TARGET_OFFSET" -ne 0 ]; then
                echo "Target Offset applied: $TARGET_OFFSET ps -> Shifted Avg: $AVERAGE ps"
            fi
            echo "Error: $ERROR ps | Accumulator: $INTEGRAL_ACCUM ps"
            echo "Applying P-Term: $P_TERM ps | I-Term: $I_TERM ps"
            echo "Total adjustment: $CORRECTIONscaled ps"
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