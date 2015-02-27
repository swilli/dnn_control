from sensor_data_loader import load_sensor_files
from sklearn.linear_model import LinearRegression
import numpy as np
from sklearn import datasets, linear_model

# Load the diabetes dataset
diabetes = datasets.load_diabetes()


# Use only one feature
diabetes_X = diabetes.data[:, np.newaxis]
diabetes_X_temp = diabetes_X[:, :, 2]

# Split the data into training/testing sets
diabetes_X_train = diabetes_X_temp[:-20]
diabetes_X_test = diabetes_X_temp[-20:]

from sklearn.datasets.samples_generator import make_regression

# this is our test set, it's just a straight line with some
# gaussian noise
X, Y = make_regression(n_samples=100, n_features=1, n_informative=1,\
                        random_state=0, noise=35)


# Split the targets into training/testing sets
diabetes_y_train = diabetes.target[:-20]
diabetes_y_test = diabetes.target[-20:]

data_path = "/home/willist/Documents/dnn/data/labeled/"

training_set, training_labels, test_set, test_labels = load_sensor_files(data_path, shared=False)

regr = LinearRegression()

regr.fit(training_set, training_labels)

print ("Residual sum of squares: %.5f" % np.mean((regr.predict(test_set) - test_labels) ** 2))