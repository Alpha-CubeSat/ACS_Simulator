The EKF has been consolidated into a single hpp file, the previous implementations of it have not been removed, but rather given a different extension of "_obsolete_". The previous logic has been maintained, but new functions have been added to the EKF to provide complete integration. 

# The EKF now handles:

 * Added state prediction and error correction capabilities

 * Added csv file calibration

 * Added state history tracking

 * Implemented offset calculation and storage

 * Added mechanisms to use past predictions for error correction