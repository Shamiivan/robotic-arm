import numpy as np
import matplotlib.pyplot as plt

x = [300,245,170,145,130,122,112,103,93,87,80,75,70,67,62,59,57]
y = [20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100]

# Finding the polynomial coefficients for a cubic polynomial
coefficients = np.polyfit(x, y, 3)
poly = np.poly1d(coefficients)

# Generating x values for the curve
x_curve = np.linspace(min(x), max(x), 100)
y_curve = poly(x_curve)

# Generating x values for extrapolation
x_extrapolate = np.linspace(0, 400, 100)  # Define x values for extrapolation
y_extrapolate = poly(x_extrapolate)  # Calculate y values using the cubic polynomial equation

# Plotting the points, polynomial curve, and extrapolation
plt.figure(figsize=(8, 6))
plt.scatter(x, y, label='Data Points')
plt.plot(x_curve, y_curve, 'r', label='Cubic Polynomial Curve')
plt.plot(x_extrapolate, y_extrapolate, 'g--', label='Extrapolation')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Cubic Polynomial Curve Fitting with Extrapolation')
plt.legend()
plt.grid(True)
plt.show()
