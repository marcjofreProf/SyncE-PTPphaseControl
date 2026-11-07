#!/bin/bash
# Bash script to automatically execute the PI phase adjustment algorithm for SynE-PTP
# Usage: 
#   ./BashPhaseAdjAutoScript.sh             (Runs continuously, 15s interval, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 30          (Runs continuously, 30s interval, target is 0 ps)
#   ./BashPhaseAdjAutoScript.sh 15 500      (Runs continuously, 15s interval, locks phase at +500 ps)

clear

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
N=60
TRIM_COUNT=4
psCLK_OUTperiod=$(( 4000 * 10 ))
psCLK_OUTperiodHalf=$((psCLK_OUTperiod/2))

# --- FPGA Hardware Offset Configuration ---
AXI_PHASE_ADDR="0x43C00000"
AXI_PPS_TS_ADDR="0x43C00004"
AXI_CLKOUT_TS_ADDR="0x43C00008"
AXI_VALID_ADDR="0x43C0000C"

PS_PER_TICK=10000
PS_PER_TICK_factor=1
M3_FILTER_N=10
M3_FILTERED=0

# --- PI Controller Tuning ---
scaled_PID_factor=1000
scaled_PIDp=550
scaled_PIDi=25

INTEGRAL_ACCUM=0

# ==========================================
# --- PPS Activation Function ---
# ==========================================
ensure_pps_activated() {
    if [ -f /sys/class/ptp/ptp0/period ]; then
        # Read the file with a simple redirect. No cat.
        read -r CURRENT_PERIOD < /sys/class/ptp/ptp0/period 2>/dev/null
        
        # If the file is not empty and not the "disabled" string, it's active
        if [[ -n "$CURRENT_PERIOD" ]] && [[ "$CURRENT_PERIOD" != "0 0 0 0 0" ]]; then
            return 0 # PPS is already running, do nothing
        fi
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

    # 5. Calculate the NEXT ABSOLUTE MINUTE BOUNDARY + offset
    START_TIME=$(( (PHC_TIME + 60) / 60 * 60 ))
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

    if [ "$ROUNDED_PS" -ne "$TARGET_OFFSET" ]; then
        echo "Note: TARGET_OFFSET=${TARGET_OFFSET}ps rounded to ${ROUNDED_PS}ps."
    fi
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
        sleep 1.2

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
            if [ "$M3_RAW_PS" -gt "$psCLK_OUTperiodHalf" ]; then
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

            curr_axi_ps=$(( curr_tick * 2081 / 100 ))
            curr_axi_ps=$(( curr_axi_ps % psCLK_OUTperiod ))
            if [ "$curr_axi_ps" -gt "$psCLK_OUTperiodHalf" ]; then
                curr_axi_ps=$(( curr_axi_ps - psCLK_OUTperiod ))
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

        M3_FILTERED=$M3_AVG
        ERROR=$(( M3_FILTERED - 0 ))

        ERROR=$(( ERROR % psCLK_OUTperiod ))
        if [ "$ERROR" -le -$psCLK_OUTperiodHalf ]; then
            ERROR=$(( ERROR + psCLK_OUTperiod ))
        elif [ "$ERROR" -gt "$psCLK_OUTperiodHalf" ]; then
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
        if [ "$CORRECTIONscaled" -le -$psCLK_OUTperiodHalf ]; then
            CORRECTIONscaled=$(( CORRECTIONscaled + psCLK_OUTperiod ))
        elif [ "$CORRECTIONscaled" -gt "$psCLK_OUTperiodHalf" ]; then
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
            if [ ${#M3_SAMPLES[@]} -gt $(( TRIM_COUNT * 2 )) ]; then
                echo "M3 samples: ${#M3_SAMPLES[@]} valid, trimmed to $(( ${#M3_SAMPLES[@]} - 2 * TRIM_COUNT ))."
            fi
            echo "PPS vs CLK_OUT (M3):     $M3_AVG ps"
            echo "SyncE TX/RX (Diag):      $AXI_TICKS ticks -> $AXI_AVG ps"

            if [ "$TARGET_OFFSET" -ne 0 ]; then
                echo "Target Setpoint Offset:  $TARGET_OFFSET ps (rounded to $ROUNDED_PS ps)"
                echo "Hardware resolution:     ${RESOLUTION_PS} ps (8 ns)"
            fi

            echo "-----------------------"
            echo "Error: $ERROR ps | Accumulator: $INTEGRAL_ACCUM ps"
            echo "Applying P-Term: $P_TERM ps | I-Term: $I_TERM ps"
            echo "Total calculated step: $CORRECTIONscaled ps"
            echo "======================="
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