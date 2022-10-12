# Script that anylizes the synchronization of pulses.
# It supports csv files with 3 coloumns:
# time,sig1,sig2
# where sig1 and sig2 can be "0" or "1".
# This is typically the file exported by logic analyzers.
# First row is ignored.
# Example of file content:
#
# Time [s],tx/rx req,fsm 0
# 0.000000000,0,0
# 0.204029950,1,0
# 0.204032150,1,1
# 0.204341150,0,1
# 0.204344100,0,0
# 0.454030300,1,0
# 0.454033300,1,1
# 0.454342250,0,1


import csv
import numpy
import argparse

TIME, SIG1, SIG2 = range(3)

tolerance = 0.1 # seconds

timeline = []
error = []

parser = argparse.ArgumentParser(description='Calculate statistics for synchronized signals.')
parser.add_argument('filename',metavar='filename', help='Name of the file in CSV format (first row is discarded)')
args = parser.parse_args()
filename = args.filename

with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=',', skipinitialspace=True)
    i = 0
    for row in csvreader:
        i += 1
        if i == 1:
            continue # Discard first row

        timestamp = float(row[0])
        master = int(row[1])
        slave = int(row[2])

        timeline.append({TIME:timestamp,SIG1:master,SIG2:slave})

def transition_at_index(signal, i):

    if timeline[i-1][signal] == 0 and timeline[i][signal]:
        return True
    return False


def search_sig2_transition(sig1_index):
    """
        sig1_index is the index where there is a sig1 transition
    """

    # Search slave transition

    for i in range(sig1_index-2, sig1_index+3): # Search from sig_index-1 to sig_index+1
        
        if transition_at_index(SIG2, i):
            delta = timeline[i][TIME] - timeline[sig1_index][TIME]
            if abs(delta) < tolerance:
                return delta
            
    print("Something wrong at time %f."%timeline[sig1_index][TIME])
    
    return None # Not found


for i in range(1,len(timeline)-2):
    # Search for master signal going high

    if timeline[i-1][SIG1] == 0 and timeline[i][SIG1] == 1:
        # Transition from 0 to 1

        delta = search_sig2_transition(i)
        if delta is not None:
            error.append(delta)

error_abs = [abs(e) for e in error]
#print(["%.3f"%(e*1e6) for e in error])

print("\n--- STATISTICS ---\n")
print("Transitions: %d.\n"%len(error))

print("Mean: %.2f us."%(numpy.mean(error)*1e6))
print("Minimum: %.2f us (index %d)."%(numpy.min(error)*1e6, numpy.argmin(error)))
print("Maximum: %.2f us (index %d)."%(numpy.max(error)*1e6, numpy.argmax(error)))
print("Standard deviation: %.2f us.\n"%(numpy.std(error)*1e6))


import matplotlib.pyplot as plt

error_us = [e*1e6 for e in error]

plt.hist(error_us, bins='auto')  # arguments are passed to np.histogram
plt.title("Offset distribution")
plt.xlabel("Offset (us)")
plt.show()

    

    


