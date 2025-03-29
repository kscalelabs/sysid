import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

def main():
    # Load CSV file with extra spaces in header handled
    data = pd.read_csv("sts3250_ErrorGain_Measurements.csv", skipinitialspace=True)
    x = data['pos_err'].values
    y = data['error_gain'].values

    # Sort the data by x (required for spline fitting)
    sorted_indices = np.argsort(x)
    x = x[sorted_indices]
    y = y[sorted_indices]

    # Fit a cubic spline (k=3). s=0 forces the spline to interpolate the data exactly.
    spline = UnivariateSpline(x, y, s=0, k=3)

    # Generate a dense grid for plotting a smooth spline curve
    x_dense = np.linspace(x.min(), x.max(), 500)
    y_dense = spline(x_dense)

    # Compute the RMSE on the original data points
    y_fit = spline(x)
    print(y_fit)
    rmse = np.sqrt(np.mean((y - y_fit) ** 2))
    print("Root Mean Square Error (RMSE):", rmse)

    # Plot the original data and the cubic spline fit
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, 'o', label="Measured")
    plt.plot(x_dense, y_dense, label="Cubic Spline Fit")
    plt.xlabel("pos_err [rad]")
    plt.ylabel("error_gain")
    plt.title("STS3250: Error Gain vs Position Error")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
