#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "Dense"

using namespace std;

class GNB {
public:

	vector<string> possible_labels = {"left","keep","right"};
	
	Eigen::ArrayXd left_means;
	Eigen::ArrayXd left_sds;
	double left_prior;
	
	Eigen::ArrayXd keep_means;
	Eigen::ArrayXd keep_sds;
	double keep_prior;
	
	Eigen::ArrayXd right_means;
	Eigen::ArrayXd right_sds;
	double right_prior;


	/**
  	* Constructor
  	*/
 	GNB();

	/**
 	* Destructor
 	*/
 	virtual ~GNB();

 	void train(vector<vector<double> > data, vector<string>  labels);

  	string predict(vector<double>);

};

#endif
