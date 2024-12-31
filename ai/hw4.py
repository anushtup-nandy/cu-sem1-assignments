from sklearn.svm import SVC
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.ensemble import AdaBoostClassifier
from sklearn.metrics import accuracy_score
from matplotlib import colormaps
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


class Classifiers():
   def __init__(self,data):
       '''
       TODO: Write code to convert the given pandas dataframe into training and testing data
       # all the data should be nxd arrays where n is the number of samples and d is the dimension of the data
       # all the labels should be nx1 vectors with binary labels in each entry
       '''
       self.training_data = None
       self.training_labels = None
       self.testing_data = None
       self.testing_labels = None
       self.outputs = []
       data = pd.read_csv('input.csv')
       X = data[['A', 'B']].to_numpy()
       y = data[['label']].to_numpy().flatten()
       self.training_data, self.testing_data, self.training_labels, self.testing_labels = train_test_split(X, y,test_size=0.4, random_state=50)
  
   def test_clf(self, clf, classifier_name=''):
       # TODO: Fit the classifier and extract the best score, training score and parameters
       clf.fit(self.training_data, self.training_labels)
       best_parameters = clf.best_params_
       best_training_score = clf.best_score_
    #    best_training_score = clf.score(self.training_data, self.training_labels)
       predicted_labels = clf.predict(self.testing_data)
       test_score = accuracy_score(self.testing_labels, predicted_labels)
       self.outputs.append(f"{classifier_name}, {best_training_score}, {test_score}")

       # Use the following line to plot the results
       self.plot(self.testing_data, clf.predict(self.testing_data),model=clf, classifier_name= classifier_name)

   def classifyNearestNeighbors(self):
       # TODO: Write code to run a Nearest Neighbors classifier
       model_pipe = KNeighborsClassifier()
       parameters = {'n_neighbors': list(range(1,20,2)), 'leaf_size': list(range(5,30,5))}
       clf = GridSearchCV(model_pipe, parameters, cv = 5, scoring ='accuracy')
       self.test_clf(clf, classifier_name='K_NN')
      
   def classifyLogisticRegression(self):
       # TODO: Write code to run a Logistic Regression classifier
       model_pipe = LogisticRegression()
       parameters = {'C': [0.1, 0.5, 1, 5, 10, 50, 100]}
       clf = GridSearchCV(model_pipe, parameters, cv = 5, scoring ='accuracy')
       self.test_clf(clf, classifier_name='Logistic_Regression')

   def classifyDecisionTree(self):
       # TODO: Write code to run a Logistic Regression classifier
       model_pipe = DecisionTreeClassifier()
       parameters = {'max_depth': list(range(1, 51)), 'min_samples_split': list(range(2,11))}
       clf = GridSearchCV(model_pipe, parameters, cv = 5, scoring ='accuracy')
       self.test_clf(clf, classifier_name='Decision Tree')

   def classifyRandomForest(self):
       # TODO: Write code to run a Random Forest classifier
       model_pipe = RandomForestClassifier()
       parameters = {'max_depth': list(range(1,6)), 'min_samples_split': list(range(2,11))}
       clf = GridSearchCV(model_pipe, parameters, cv = 5, scoring ='accuracy')
       self.test_clf(clf, classifier_name='Random Forest')


   def classifyAdaBoost(self):
       # TODO: Write code to run a AdaBoost classifier
       model_pipe = AdaBoostClassifier()
       parameters = {'n_estimators': [10,20,30,40,50,60,70]}
       clf = GridSearchCV(model_pipe, parameters, cv = 5, scoring ='accuracy')
       self.test_clf(clf, classifier_name='AdaBoost')
       pass


   def plot(self, X, Y, model,classifier_name = ''):
    #    X1 = X.iloc[:, 0]
    #    X2 = X.iloc[:, 1]
       X1 = X[:, 0]
       X2 = X[:, 1]


       X1_min, X1_max = min(X1) - 0.5, max(X1) + 0.5
       X2_min, X2_max = min(X2) - 0.5, max(X2) + 0.5


       X1_inc = (X1_max - X1_min) / 200.
       X2_inc = (X2_max - X2_min) / 200.


       X1_surf = np.arange(X1_min, X1_max, X1_inc)
       X2_surf = np.arange(X2_min, X2_max, X2_inc)
       X1_surf, X2_surf = np.meshgrid(X1_surf, X2_surf)


       L_surf = model.predict(np.c_[X1_surf.ravel(), X2_surf.ravel()])
       L_surf = L_surf.reshape(X1_surf.shape)


       plt.title(classifier_name)
       plt.contourf(X1_surf, X2_surf, L_surf, cmap = plt.cm.coolwarm, zorder = 1)
       plt.scatter(X1, X2, s = 38, c = Y)


       plt.margins(0.0)
       # uncomment the following line to save images
       plt.savefig(f'{classifier_name}.png')
       plt.show()


  
if __name__ == "__main__":
   df = pd.read_csv('input.csv')
   models = Classifiers(df)
   print('Classifying with NN...')
   models.classifyNearestNeighbors()
   print('Classifying with Logistic Regression...')
   models.classifyLogisticRegression()
   print('Classifying with Decision Tree...')
   models.classifyDecisionTree()
   print('Classifying with Random Forest...')
   models.classifyRandomForest()
   print('Classifying with AdaBoost...')
   models.classifyAdaBoost()


   with open("output.csv", "w") as f:
       print('Name, Best Training Score, Testing Score',file=f)
       for line in models.outputs:
           print(line, file=f)
