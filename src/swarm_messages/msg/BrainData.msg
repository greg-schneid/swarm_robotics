# One EEG sample (or a small chunk if you choose)
# Timestamp in seconds (LSL local_clock() or inlet ts)
float64 timestamp

# EEG values in μV (Muse order: TP9, AF7, AF8, TP10, AUX)
# Shape (C,) where C is the number of channels
float32[] values
