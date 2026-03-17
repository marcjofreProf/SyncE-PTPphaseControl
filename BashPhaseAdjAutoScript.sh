#!/bin/bash
# Bash scrip to automatically execute the phase adjustment algorithm for SynE-PTP

# --- Configuration ---
INTERFACE="eth0"
N=20                # Number of samples to average
SUM=0
psCLK_OUTperiod=100000      # Period of the CLK_OUT signal in picoseconds

psCLK_OUTperiodHalf=$((psCLK_OUTperiod/2))

scaled_PIDksmall=95       # PID proportional constant. Since decimal numbers is difficult to work with, the PID constant is scaled  to have integer values
scaled_PIDklarge=$scaled_PIDksmall       # PID proportional constant, when we have to cover the high part
scaled_PID_factor=100      # Scaling value to operate with integers

#echo "Starting SyncE-PTP Phase Adjustment ($N samples)..."

valArray=()
HAS_LARGE=0

# Loop N times to collect samples
for (( i=1; i<=$N; i++ ))
do
    # Run phc_ctl and extract the numeric value (assuming output is like "offset: 123")

    sudo phc_ctl $INTERFACE -- phaseadj 0

    # Give time to return the info thorugh the kernel and pause a bit the loop
    sleep 0.5

    # We look for our prefix, take the last line, and extract the number
    val=$(dmesg | grep "PHC_PHASE_RESULT:" | tail -1 | awk -F': ' '{print $NF}')
    
    if [[ -z "$val" ]]; then
        echo "Error: Could not read offset from $INTERFACE at sample $i"
        continue
    fi
    # Store in array
    valArray[$i]=$val

    if [ "$val" -gt "$psCLK_OUTperiodHalf" ]; then
        HAS_LARGE=1
    fi
done

# Sum them up - the "Apples to Apples" part
for v in "${valArray[@]}"; do
    # If we have large values in the batch, move the small ones up
    if [ "$HAS_LARGE" -eq 1 ] && [ "$v" -lt "$psCLK_OUTperiodHalf" ]; then
        SUM=$(( SUM + v + psCLK_OUTperiod ))
    else
        SUM=$(( SUM + v ))
    fi
done

# Calculate Average and ensure it's within [0, fullperiod]
AVERAGE=$(( SUM / N ))

# Final check to keep it positive and within one period
if [ "$AVERAGE" -ge "$psCLK_OUTperiod" ]; then
    AVERAGE=$(( AVERAGE - psCLK_OUTperiod ))
fi

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

#echo "Applying correction: $CORRECTION ns to $INTERFACE"
sudo phc_ctl $INTERFACE -- phaseadj $CORRECTION

#echo "Adjustment Complete."
