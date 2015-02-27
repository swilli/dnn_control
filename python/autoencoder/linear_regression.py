from sensor_data_loader import load_sensor_files
from sklearn.linear_model import LinearRegression
import numpy as np

data_path = "/home/willist/Documents/dnn/data/labeled/"

training_set, training_labels, test_set, test_labels = load_sensor_files(data_path, shared=False)

regr = LinearRegression()

regr.fit(training_set, training_labels)

predicted_labels = regr.predict(test_set)
print ("Residual sum of squares: %.5f" % np.mean(np.sum((predicted_labels - test_labels) ** 2, axis=1)))