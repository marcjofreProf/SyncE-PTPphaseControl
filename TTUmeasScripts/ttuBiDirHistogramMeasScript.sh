#!/bin/bash
# Bash script to automate the Swabian Time Tagger API with Live Plotting
# Usage: ./measure_tt.sh <OFFSET_IN_PS>

echo "========================================================="
echo "  Checking dependencies and installing if needed...      "
echo "========================================================="

if ! dpkg -s python3-pip >/dev/null 2>&1; then
    echo "Installing python3-pip..."
    sudo apt-get update
    sudo apt-get install -y python3-pip
fi

if ! dpkg -s libxcb-xinerama0 >/dev/null 2>&1; then
    echo "Installing missing GUI libraries for Qt5..."
    sudo apt-get update
    sudo apt-get install -y libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
fi

if ! python3 -c "import matplotlib" &> /dev/null; then
    echo "Installing Python module: matplotlib..."
    pip3 install matplotlib
fi

if ! python3 -c "import PyQt5" &> /dev/null; then
    echo "Installing Python module: PyQt5 (for GUI backend)..."
    pip3 install PyQt5
fi

if ! python3 -c "import Swabian.TimeTagger" &> /dev/null; then
    echo "Installing Python module: Swabian-TimeTagger..."
    pip3 install Swabian-TimeTagger
fi

echo "All dependencies satisfied."

echo "Checking for background Time Tagger servers..."
pkill -f "TimeTaggerWeb" 2>/dev/null
pkill -f "TimeTagger" 2>/dev/null

echo "Allowing USB interface to reset..."
sleep 3 

# =========================================================
#   MEASUREMENT PARAMETERS (SINGLE SOURCE OF TRUTH)
# =========================================================
OFFSET_PS=${1:-0}
TRIGGER_VOLTAGE=1.5
DURATION_S=10
BINWIDTH_PS=25
N_BINS=1000

OUTPUT_DIR="../../TTUdataMeas"
OUTPUT_FILE="${OUTPUT_DIR}/histogram_offset_${OFFSET_PS}ps.txt"
PLOT_FILE="${OUTPUT_DIR}/histogram_offset_${OFFSET_PS}ps.png"

mkdir -p "$OUTPUT_DIR"

export QT_QPA_PLATFORM=xcb

echo "========================================================="
echo "  Starting Correlation Measurement - Time Tagger 20      "
echo "========================================================="
echo "Channels: 2 and 3 | Offset: $OFFSET_PS ps | Thresholds: $TRIGGER_VOLTAGE V"
echo "Integration Time: $DURATION_S seconds"
echo "Output File: $OUTPUT_FILE"
echo "Plot File: $PLOT_FILE"
echo "========================================================="

# --- 1. MAIN MEASUREMENT SCRIPT ---
python3 - <<EOF
import sys
import time
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import Swabian.TimeTagger as tt

delay_offset = int("$OFFSET_PS")
output_file = "$OUTPUT_FILE"
plot_file = "$PLOT_FILE"

ch_start = 2
ch_stop = 3

# Reading parameters injected from Bash
trigger_voltage = float("$TRIGGER_VOLTAGE")
binwidth_ps = int("$BINWIDTH_PS")
n_bins = int("$N_BINS")
duration_s = int("$DURATION_S")

tagger = None
max_retries = 3

for attempt in range(max_retries):
    try:
        print(f"-> Attempting to connect to Time Tagger (Try {attempt + 1}/{max_retries})...")
        tagger = tt.createTimeTagger()
        print("-> Connection successful!")
        break
    except Exception as e:
        print(f"   [!] Connection failed: {e}")
        if attempt < max_retries - 1:
            print("   [*] Retrying in 2 seconds...")
            time.sleep(2)
        else:
            print("ERROR: Could not connect after multiple attempts. A physical USB plug cycle is required.")
            sys.exit(1)

try:
    # --- HARDWARE CONFIGURATION ---
    print(f"-> Setting trigger levels for Channels {ch_start} and {ch_stop} to {trigger_voltage}V...")
    tagger.setTriggerLevel(ch_start, trigger_voltage)
    tagger.setTriggerLevel(ch_stop, trigger_voltage)
    
    tagger.setDelaySoftware(ch_stop, delay_offset)
    
    corr = tt.Correlation(tagger, channel_1=ch_start, channel_2=ch_stop, binwidth=binwidth_ps, n_bins=n_bins)
    
    print(f"-> Acquiring data for {duration_s} seconds. Opening live plot window...")
    corr.startFor(int(duration_s * 1e12))
    
    plt.ion() 
    fig, ax = plt.subplots(figsize=(10, 6))
    
    line, = ax.step(corr.getIndex(), corr.getData(), where='mid', color='blue', alpha=0.8)
    
    ax.set_title(f"Live Correlation (Offset: {delay_offset} ps)")
    ax.set_xlabel("Time (ps)")
    ax.set_ylabel("Counts")
    ax.grid(True, linestyle='--', alpha=0.6)
    plt.show()
    
    while corr.isRunning():
        line.set_ydata(corr.getData())
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.1)
        
    print("-> Acquisition finished. Saving data...")
    plt.ioff()
    plt.close(fig)
    
    with open(output_file, "w") as f:
        f.write("Time(ps)\tCounts\n")
        for t, c in zip(corr.getIndex(), corr.getData()):
            f.write(f"{int(t)}\t{int(c)}\n")
            
    print(f"-> SUCCESS: Data successfully saved to '{output_file}'")
    
    # Save the plot with the threshold in the title for the PNG as well
    plt.figure(figsize=(10, 6))
    plt.step(corr.getIndex(), corr.getData(), where='mid', color='blue', alpha=0.8)
    plt.title(f'Correlation (Offset: {delay_offset} ps, Threshold: {trigger_voltage} V)')
    plt.xlabel('Time (ps)')
    plt.ylabel('Counts')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.savefig(plot_file)
    print(f"-> SUCCESS: Plot image successfully saved to '{plot_file}'")

    print("-> Releasing Time Tagger hardware immediately...")
    tt.freeTimeTagger(tagger)
    tagger = None
    time.sleep(1)
    print("-> Hardware successfully released.")

except Exception as e:
    print(f"ERROR: Could not execute measurement. Details: {e}")
    sys.exit(1)

finally:
    if tagger is not None:
        try:
            tt.freeTimeTagger(tagger)
            time.sleep(1)
        except:
            pass
EOF

# --- 2. BACKGROUND PLOT VIEWER ---
echo "-> Launching persistent background plot..."
nohup python3 -c "
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

t_vals, c_vals = [], []
try:
    with open('$OUTPUT_FILE', 'r') as f:
        next(f) # Skip header
        for line in f:
            parts = line.strip().split('\t')
            if len(parts) == 2:
                t_vals.append(float(parts[0]))
                c_vals.append(float(parts[1]))
                
    plt.figure(figsize=(10, 6))
    plt.step(t_vals, c_vals, where='mid', color='blue', alpha=0.8)
    plt.title('Final Correlation (Offset: $OFFSET_PS ps, Threshold: $TRIGGER_VOLTAGE V)')
    plt.xlabel('Time (ps)')
    plt.ylabel('Counts')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.show()
except Exception:
    pass
" >/dev/null 2>&1 &

echo "Script execution completed. You can now use your terminal."