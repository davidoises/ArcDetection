import pandas as pd
import numpy as np
from sklearn import svm, metrics
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from os import path
import sys


# FEATURE_INDICES = np.array([ 6, 11, 17, 31, 86]) # double window differences
FEATURE_INDICES = np.array([ 93,  96, 102, 104, 174]) # double window
DISPLAY_CONFUSION = False

print("STARTING")
print("\r\nLOADING INPUT DATASET\r\n")
basepath = path.dirname(sys.argv[0])
processed_dataset = path.join(basepath, "preprocessed_data/processed_data.csv")
df = pd.read_csv(processed_dataset)


X = df.drop(['conditions', 'time', 'arc'], axis=1)
X = X.iloc[:, FEATURE_INDICES]
y = df['arc']
X_train, X_test,y_train, y_test = train_test_split(X.to_numpy(), y.to_numpy(), test_size=0.2, random_state=42)
print("Input features are: ")
print(X.columns)


print("\r\nTRAINING MODEL\r\n")
regressor = svm.SVC(kernel='linear')
regressor.fit(X_train,y_train)
print("Model weights are: ")
print(regressor.coef_)
print(regressor.intercept_)


print("\r\nEVALUATING MODEL\r\n")
test_prediction = regressor.predict(X_test)
train_prediction = regressor.predict(X_train)
confusion_matrix = metrics.confusion_matrix(y_test, test_prediction)

print(confusion_matrix)

if (DISPLAY_CONFUSION) :
    cm_display = metrics.ConfusionMatrixDisplay(confusion_matrix = confusion_matrix, display_labels = ['no arc', 'arc'])
    cm_display.plot()
    plt.show()

print("\r\nTESTING MODEL\r\n")

print("Test 1: ")
print("Sample: ")
X = X_test[:1]
y = y_test[:1]
print(X)
print(y)
print("result: ")
# y_pred_lib = regressor.predict(X)
y_pred_lib = regressor.decision_function(X)
res = np.dot(X, np.transpose(regressor.coef_)) + regressor.intercept_

print("Sklearn result: ", y_pred_lib)
print("Manual operation result: ", res)


print("\r\nTest 2: ")
X = X_test[3:4]
y = y_test[3:4]
print(X)
print(y)

print("result: ")
# y_pred_lib = regressor.predict(X)
y_pred_lib = regressor.decision_function(X)
res = np.dot(X, np.transpose(regressor.coef_)) + regressor.intercept_

print("Sklearn result: ", y_pred_lib)
print("Manual operation result: ", res)
