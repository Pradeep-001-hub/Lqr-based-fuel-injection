MATLAB FUNCTIONS USED AND THEIR DESCRIPTION

The implementation of the Kalman Filter for AFR and RPM estimation uses several important MATLAB functions and operations that enable efficient simulation and numerical computation.

sqrtm()
The sqrtm function computes the matrix square root. It is used to generate properly scaled Gaussian noise based on covariance matrices Q and R. This ensures that the simulated process noise and measurement noise follow the correct statistical properties required for realistic system modeling.

randn()
The randn function generates normally distributed random numbers with zero mean and unit variance. In this code, it is used along with sqrtm to create stochastic noise affecting both the system dynamics and measurements.

eye()
The eye function creates an identity matrix. It is used in defining system matrices, initializing the covariance matrix P, and in the Kalman update equation to maintain correct matrix dimensions and structure.

zeros()
The zeros function initializes matrices with all elements set to zero. It is used for preallocating memory for storing state history (x_hist and xhat_hist), improving computational efficiency by avoiding dynamic resizing inside loops.

length()
The length function returns the size of the time vector. It is used to control the simulation loop and ensure all time steps are processed correctly.

plot()
The plot function is used to visualize the results. It plots true vs estimated AFR and RPM over time, allowing comparison between actual system behavior and Kalman Filter estimation.

legend()
The legend function labels the plotted curves, helping distinguish between true and estimated values in the graphs.

xlabel() and ylabel()
These functions label the x-axis and y-axis of the plots. They improve readability and clearly indicate the physical meaning of each axis (time, AFR, RPM).

title()
The title function provides a heading for each plot, describing what is being visualized.

grid on
This command enables grid lines in the plots, making it easier to interpret variations and compare signals.

matrix transpose (')
The transpose operator is used in equations such as P*A' and C'. It is essential for correct matrix multiplication in Kalman Filter equations.

matrix division (/)
The right division operator is used in calculating the Kalman Gain K = P*C'/S. It provides a numerically stable way to solve matrix equations without explicitly computing the inverse.

OVERALL IMPORTANCE

These MATLAB functions together enable accurate simulation, efficient computation, and clear visualization of the Kalman Filter-based estimation process. They ensure that the implementation is both mathematically correct and computationally optimized, which is essential for real-time control and estimation applications.