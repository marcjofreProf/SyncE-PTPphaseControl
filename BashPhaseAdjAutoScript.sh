#!/bin/bash
# Bash scrip to automatically execute the phase adjustment algorithm for SynE-PTP
# To call the script iteratively (for isntance every 120 s) run it as watch -n 120 ./BashPhaseAdjAutoScript.sh

# --- Configuration ---
PlotInfo=false       # Plot extra information ot the user: true or false
INTERFACE="eth0"
N=20                # Number of samples to average
psCLK_OUTperiod=100000      # Period of the CLK_OUT signal in picoseconds

psCLK_OUTperiodHalf=$((psCLK_OUTperiod/2))

scaled_PIDksmall=100       # PID proportional constant
scaled_PIDklarge=$scaled_PIDksmall       
scaled_PID_factor=100      # Scaling value to operate with integers

# --- Runtime Variables ---
SUM=0
VALID_SAMPLES=0
wrap_offset=0
prev_val=""

#echo "Starting SyncE-PTP Phase Adjustment ($N samples)..."

# Loop N times to collect samples
for (( i=1; i<=$N; i++ ))
do
    # Pause a bit the loop so that ptp4l does not suffer from errors
    sleep 1.9 

    # Trigger a 0-adjustment read in the C driver
    sudo phc_ctl $INTERFACE -- phaseadj 0 > /dev/null 2>&1

    # Give time to return the info through the kernel
    sleep 0.1

    # Extract the number
    val=$(dmesg | grep "PHC_PHASE_RESULT:" | tail -1 | awk -F': ' '{print $NF}')
    
    if [[ -z "$val" ]]; then
        echo "Error: Could not read offset from $INTERFACE at sample $i"
        continue
    else
        if [ "$PlotInfo" = "true" ]; then
            echo "PHC_PHASE_RESULT: $val ps"
        fi
    fi

    # FIX 1: Sequential Unwrapping (The true "Apples to Apples")
    if [ "$VALID_SAMPLES" -eq 0 ]; then
        prev_val=$val
        unwrapped_val=$val
    else
        diff=$(( val - prev_val ))
        # Wrapped forward (e.g., 99000 -> 1000)
        if [ "$diff" -lt "-$psCLK_OUTperiodHalf" ]; then
            wrap_offset=$(( wrap_offset + psCLK_OUTperiod ))
        # Wrapped backward (e.g., 1000 -> 99000)
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
    echo "Error: No valid samples collected. Aborting adjustment."
    exit 1
fi

# Calculate Average based on actual successful reads
AVERAGE=$(( SUM / VALID_SAMPLES ))

# FIX 2: Strip away the wrap_offset to get back into [0, fullperiod] domain
AVERAGE=$(( AVERAGE % psCLK_OUTperiod ))
if [ "$AVERAGE" -lt 0 ]; then
    AVERAGE=$(( AVERAGE + psCLK_OUTperiod ))
fi

# --- P-Controller Math ---
# Apply the correction (it should be provided in picoseconds)
if [ "$AVERAGE" -gt "$psCLK_OUTperiodHalf" ]; then
    CORRECTIONscaled=$(((scaled_PIDklarge * (AVERAGE - psCLK_OUTperiod)) / (scaled_PID_factor)))
else
    CORRECTIONscaled=$(((scaled_PIDksmall * AVERAGE) / (scaled_PID_factor)))
fi

if [ "$CORRECTIONscaled" -ge "$psCLK_OUTperiod" ]; then
    CORRECTIONscaled=$((CORRECTIONscaled - psCLK_OUTperiod))
elif [ "$CORRECTIONscaled" -le "-$psCLK_OUTperiod" ]; then
    CORRECTIONscaled=$((CORRECTIONscaled + psCLK_OUTperiod))
fi

ABS_CORRECTION=${CORRECTIONscaled#-}

# Determine the sign
if [[ "$CORRECTIONscaled" == -* ]]; then
    SIGN="-"
else
    SIGN=""
fi

# We use the absolute value for padding to avoid "-" inside the zeros
CORRECTION=$(printf -- "%s0.%012d" "$SIGN" "$ABS_CORRECTION")

#echo "Applying correction: $CORRECTION seconds to $INTERFACE"
sudo phc_ctl $INTERFACE -- phaseadj $CORRECTION

#echo "Adjustment Complete."
