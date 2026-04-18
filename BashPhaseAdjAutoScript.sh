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
N=15                # Number of samples to collect per interval
TRIM_COUNT=4        # Trim average: Discard this many highest and lowest samples (e.g., 3 removes top 3 and bottom 3)

# Original Macro Period Constraints
ORIG_PERIOD=$(( 4000 * 10 ))          # 40,000 ps
ORIG_HALF=$(( ORIG_PERIOD / 2 ))      # 20,000 ps

# Active Controller Constraints (Starts in Macro-Lock)
psCLK_OUTperiod=$ORIG_PERIOD
psCLK_OUTperiodHalf=$ORIG_HALF

# --- PI Controller Tuning ---
scaled_PID_factor=1000      # Scaling value to operate with integers
scaled_PIDp=600             # Proportional gain (0.60) - Pulls aggressively to 0
scaled_PIDi=15              # Integral gain (0.015) - Corrects static drift

# --- Initialize Persistent Memory ---
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

    clear
    UNWRAPPED_SAMPLES=()
    VALID_SAMPLES=0
    wrap_offset=0
    prev_val=""

    # Loop N times to collect samples
    for (( i=1; i<=$N; i++ ))
    do
        sudo dmesg -c > /dev/null
        
        sleep 0.20
        sudo phc_ctl $INTERFACE -- phaseadj 0 > /dev/null 2>&1
        sleep 0.05

        val=$(dmesg | grep "PHC_PHASE_RESULT:" | tail -1 | awk -F': ' '{print $NF}')
        
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

        UNWRAPPED_SAMPLES+=("$unwrapped_val")
        VALID_SAMPLES=$(( VALID_SAMPLES + 1 ))
    done

    if [ "$VALID_SAMPLES" -eq 0 ]; then
        echo "Error: No valid samples collected. Skipping this interval."
    else
        # ==========================================
        # --- Trimmed Averaging Math ---
        # ==========================================
        if [ "$VALID_SAMPLES" -le $(( TRIM_COUNT * 2 )) ]; then
            echo "Warning: Not enough samples for trimmed average. Doing normal mean."
            TRIMMED_SUM=0
            for val in "${UNWRAPPED_SAMPLES[@]}"; do
                TRIMMED_SUM=$(( TRIMMED_SUM + val ))
            done
            RAW_AVERAGE=$(( TRIMMED_SUM / VALID_SAMPLES ))
        else
            SORTED=($(printf "%s\n" "${UNWRAPPED_SAMPLES[@]}" | sort -n))
            TRIMMED_SUM=0
            TRIMMED_COUNT=0
            for (( j=TRIM_COUNT; j<VALID_SAMPLES-TRIM_COUNT; j++ ))
            do
                TRIMMED_SUM=$(( TRIMMED_SUM + SORTED[j] ))
                TRIMMED_COUNT=$(( TRIMMED_COUNT + 1 ))
            done
            RAW_AVERAGE=$(( TRIMMED_SUM / TRIMMED_COUNT ))
        fi

        # ==========================================
        # --- Dynamic State Machine & Error Math ---
        # ==========================================

        # 1. Calculate Raw Signed Error
        RAW_ERROR=$(( RAW_AVERAGE - TARGET_OFFSET ))

        # 2. Find TRUE absolute error relative to the original 40,000 ps macro-period
        TRUE_ERROR=$(( RAW_ERROR % ORIG_PERIOD ))
        if [ "$TRUE_ERROR" -gt "$ORIG_HALF" ]; then
            TRUE_ERROR=$(( TRUE_ERROR - ORIG_PERIOD ))
        elif [ "$TRUE_ERROR" -lt "-$ORIG_HALF" ]; then
            TRUE_ERROR=$(( TRUE_ERROR + ORIG_PERIOD ))
        fi
        
        ABS_TRUE_ERROR=${TRUE_ERROR#-}

        # 3. Lock/Unlock Logic
        if [ "$psCLK_OUTperiod" -eq 8000 ] && [ "$ABS_TRUE_ERROR" -gt 8000 ]; then
            # We were in fine lock, but drifted too far! Revert to macro period.
            psCLK_OUTperiod=$ORIG_PERIOD
            psCLK_OUTperiodHalf=$ORIG_HALF
            if [ "$PlotInfo" = "true" ]; then
                echo "!!! LOSS OF LOCK: True error ($ABS_TRUE_ERROR ps) exceeded 8000 ps. Reverting to macro period ($ORIG_PERIOD ps). !!!"
            fi
        elif [ "$psCLK_OUTperiod" -eq "$ORIG_PERIOD" ] && [ "$ABS_TRUE_ERROR" -lt 8000 ]; then
            # We got close enough! Switch to fine lock to ignore hardware jumps.
            psCLK_OUTperiod=8000
            psCLK_OUTperiodHalf=4000
            if [ "$PlotInfo" = "true" ]; then
                echo ">>> FINE LOCK TRIGGERED: True error ($ABS_TRUE_ERROR ps) is < 8000 ps. Switching to fine lock (8000 ps). <<<"
            fi
        fi

        # 4. Normalize Error for the Controller using the ACTIVE period
        ERROR=$(( RAW_ERROR % psCLK_OUTperiod ))
        if [ "$ERROR" -gt "$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( ERROR - psCLK_OUTperiod ))
        elif [ "$ERROR" -lt "-$psCLK_OUTperiodHalf" ]; then
            ERROR=$(( ERROR + psCLK_OUTperiod ))
        fi

        # ==========================================
        # --- PI Controller ---
        # ==========================================

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
        if [ "$CORRECTIONscaled" -gt "$psCLK_OUTperiodHalf" ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled - psCLK_OUTperiod ))
        elif [ "$CORRECTIONscaled" -lt "-$psCLK_OUTperiodHalf" ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled + psCLK_OUTperiod ))
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
            echo "True Physical Error: $TRUE_ERROR ps"
            echo "Active Controller Error: $ERROR ps | Accumulator: $INTEGRAL_ACCUM ps"
            echo "Applying P-Term: $P_TERM ps | I-Term: $I_TERM ps"
            echo "Total calculated step: $CORRECTIONscaled ps"
            
            if [ "$psCLK_OUTperiod" -eq 8000 ]; then
                echo "Controller State: FINE LOCK (8000 ps period)"
            else
                echo "Controller State: MACRO LOCK ($ORIG_PERIOD ps period)"
            fi
        fi

        sudo phc_ctl $INTERFACE -- phaseadj $CORRECTION
    fi

    # --- Loop Control ---
    if [ "$INTERVAL" -eq 0 ]; then
        break
    fi

    if [ "$PlotInfo" = "true" ]; then
        echo "Sleeping for $INTERVAL seconds..."
        echo "-----------------------"
    fi
    sleep "$INTERVAL"

done