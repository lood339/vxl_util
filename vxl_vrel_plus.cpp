//
//  vxl_vrel_plus.cpp
//  CameraPlaning
//
//  Created by Jimmy Chen LOCAL on 8/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_vrel_plus.h"
#include <vrel/vrel_homography.h>
#include <vcl_map.h>
#include <vgl/vgl_fit_ellipse_2d.h>
#include "vxl_least_square.h"

/*
      *************************** VrelPlus ********************************
 */

vgl_h_matrix_2d< double > VrelPlus::homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                 vcl_vector< vgl_point_2d< double > > const& second,
                                 vcl_vector< bool > & inlier,
                                 double error_threshold)
{
    assert(first.size() >= 4);
    assert(first.size() == second.size());
    assert(inlier.size() == 0);
    
    
    vgl_h_matrix_2d< double > H;
    bool isOk = vrel_homography(first, second, H);
    
    inlier.resize(first.size());
    for (int i = 0; i<first.size(); i++) {
        const vgl_homg_point_2d<double> p1 = (vgl_homg_point_2d<double>)first[i];
        const vgl_homg_point_2d<double> p2 = (vgl_homg_point_2d<double>)second[i];
        vgl_homg_point_2d<double> proj_p1 = H(p1);
        double x = proj_p1.x()/proj_p1.w();
        double y = proj_p1.y()/proj_p1.w();
        
        double dx = x - p2.x();
        double dy = y - p2.y();
        
        double dis_2 = dx * dx + dy * dy;
        if (dis_2 <= error_threshold * error_threshold) {
            inlier[i] = true;
        }
        else
        {
            inlier[i] = false;
        }
    }
    return H;
}


bool VrelPlus::homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                                      vcl_vector< vgl_point_2d< double > > const& second,
                                                      vcl_vector< bool > & inlier, vgl_h_matrix_2d< double > & H,
                                                      double error_threshold)
{
    assert(first.size() >= 4);
    assert(first.size() == second.size());
    assert(inlier.size() == 0);
        
   
    bool isOk = vrel_homography(first, second, H);
    if(!isOk)
    {
        return false;
    }
    
    inlier.resize(first.size());
    for (int i = 0; i<first.size(); i++) {
        const vgl_homg_point_2d<double> p1 = (vgl_homg_point_2d<double>)first[i];
        const vgl_homg_point_2d<double> p2 = (vgl_homg_point_2d<double>)second[i];
        vgl_homg_point_2d<double> proj_p1 = H(p1);
        double x = proj_p1.x()/proj_p1.w();
        double y = proj_p1.y()/proj_p1.w();
        
        double dx = x - p2.x();
        double dy = y - p2.y();
        
        double dis_2 = dx * dx + dy * dy;
        if (dis_2 <= error_threshold * error_threshold) {
            inlier[i] = true;
        }
        else
        {
            inlier[i] = false;
        }
    }
    return true;
}



//select 5 points from edge points
static bool select5Points(const vcl_vector<vcl_vector<vgl_point_2d<double> > > &labelPt, vcl_vector<vgl_point_2d<double>> &arcPt)
{
	int index1 = 0;
    int index2 = 0;
	
	//select two random arc
	do
	{
		index1 = rand()%labelPt.size();
		index2 = rand()%labelPt.size();
	} while (index1 == index2 || labelPt[index1].size()<3 || labelPt[index2].size()<3);
    
    //select two point in first arc
	int id1 = 0;
    int id2 = 0;
    
	do
	{
		id1 = rand()%labelPt[index1].size();
		id2 = rand()%labelPt[index1].size();
	} while (id1 == id2);
	arcPt.push_back(labelPt[index1][id1]);
	arcPt.push_back(labelPt[index1][id2]);
    
	//select three points in second arc
    int id3 = 0;
	do
	{
		id1 = rand()%labelPt[index2].size();
		id2 = rand()%labelPt[index2].size();
		id3 = rand()%labelPt[index2].size();
	} while (id1 == id2 ||id1 == id3 || id2 == id3);
	arcPt.push_back(labelPt[index2][id1]);
	arcPt.push_back(labelPt[index2][id2]);
	arcPt.push_back(labelPt[index2][id3]);
	return true;
}

static bool select6Points(const vcl_vector<vcl_vector<vgl_point_2d<double> > > &labelPt, vcl_vector<vgl_point_2d<double>> &arcPt)
{
	int index1 = 0;
    int index2 = 0;
	
	//select two random arc
	do
	{
		index1 = rand()%labelPt.size();
		index2 = rand()%labelPt.size();
	} while (index1 == index2 || labelPt[index1].size()<3 || labelPt[index2].size()<3);
    
    //select two point in first arc
	int id1 = 0;
    int id2 = 0;
    int id3 = 0;
	do
	{
		id1 = rand()%labelPt[index1].size();
		id2 = rand()%labelPt[index1].size();
        id3 = rand()%labelPt[index1].size();
	} while (id1 == id2 || id1 == id3 || id2 == id3);
	arcPt.push_back(labelPt[index1][id1]);
	arcPt.push_back(labelPt[index1][id2]);
    arcPt.push_back(labelPt[index1][id3]);
    
	//select three points in second arc
    
	do
	{
		id1 = rand()%labelPt[index2].size();
		id2 = rand()%labelPt[index2].size();
		id3 = rand()%labelPt[index2].size();
	} while (id1 == id2 ||id1 == id3 || id2 == id3);
	arcPt.push_back(labelPt[index2][id1]);
	arcPt.push_back(labelPt[index2][id2]);
	arcPt.push_back(labelPt[index2][id3]);
	return true;
}


//fit an ellipse from arc points using least square to minimize following equation
//a*x*x + 2*b*x*y + c*y*y + 2*d*x + 2*f*y + g = 0 and a = 1
static bool fitEllipse(const vcl_vector<vgl_point_2d<double> > &arcPt, vnl_vector_fixed<double, 5> & ellipse)
{
	vcl_vector<vcl_map<int, double> > matrix_vec;
	vcl_vector<double> right_vec;
	for(int i = 0; i<arcPt.size(); ++i)
	{
		double x = arcPt[i].x();
		double y = arcPt[i].y();
		vcl_map<int, double> left;
		left[0] = 2.0*x*y;
		left[1] = y*y;
		left[2] = 2.0*x;
		left[3] = 2.0*y;
		left[4] = 1;
		matrix_vec.push_back(left);
		right_vec.push_back(-1.0*x*x);
	}
    bool isSolved = VxlLeastSquare::solver(matrix_vec, right_vec, false, 5, &ellipse[0]);    
	return isSolved;
}

//distance from point to ellipse, Sampson approximation
//multiple view geometry in computer vision, p100
static double distancePoint2ellipse(double x, double y, const vnl_vector_fixed<double, 5> & ellipseFun)
{
    /*
	cv::Mat p = cv::Mat(3, 1, CV_64F);
	p.at<double>(0, 0) = x;
	p.at<double>(1, 0) = y;
	p.at<double>(2, 0) = 1.0;
	cv::Mat C = cv::Mat(3, 3, CV_64F);
	double a = 1.0;
	double b = ellipseFun[0];
	double c = ellipseFun[1];
	double d = ellipseFun[2];
	double f = ellipseFun[3];
	double g = ellipseFun[4];
     */
    vnl_matrix<double> p(3, 1);  //vector
    p(0, 0) = x;
    p(1, 0) = y;
    p(2, 0) = 1.0;
    vnl_matrix<double> C(3, 3, 0);
    double a = 1.0;
	double b = ellipseFun[0];
	double c = ellipseFun[1];
	double d = ellipseFun[2];
	double f = ellipseFun[3];
	double g = ellipseFun[4];
    
	/*
     a b d
     b c f
     d f g
     */
	C(0, 0) = a;
	C(0, 1) = b;
	C(0, 2) = d;
	C(1, 0) = b;
	C(1, 1) = c;
	C(1, 2) = f;
	C(2, 0) = d;
	C(2, 1) = f;
	C(2, 2) = g;
    
	//cv::Mat pTCp = p.t()*C*p;
    vnl_matrix<double> pTCp= p.transpose() * C * p;
	vnl_matrix<double> Cp = C*p;  //vector
	double Cp00 = Cp(0, 0);
	double Cp10 = Cp(1, 0);
	float dis2 = (pTCp(0, 0)) * (pTCp(0, 0))/(4*(Cp00 * Cp00 + Cp10 * Cp10));
	if (dis2 > 0.0)
	{
		return sqrt(dis2);
	}
	else
	{
		return 0.0;
	}
}


bool VrelPlus::fit_ellipse_RANSAC(const vcl_vector<vcl_vector<vgl_point_2d<double> > > &labelPt,
                                  const double threshold, const double fail_ratio,
                                  vgl_ellipse_2d<double> & ellipse,
                                  vnl_vector_fixed<double, 5> & ellipseEquation,
                                  vcl_vector<vgl_point_2d<double> > & fittingPoints, int maxIter)
{
	assert(0);
	int num_max = 0;
	vcl_vector<vgl_point_2d<double>>  arc_pt_max;
    
	//total edge numbers
	int totalNum = 0;
	for (int i = 0; i<labelPt.size(); ++i)
	{
		totalNum += labelPt[i].size();
	}
    
	int iterNum = maxIter;
	for(int i = 0; i<iterNum; ++i)
	{
		//select three points from one arc and two from other
		vcl_vector<vgl_point_2d<double> > arcPt;
		select6Points(labelPt, arcPt);
        
		vnl_vector_fixed<double, 5> ellipseFun;
		//fit a ellipse
		bool isOk = fitEllipse(arcPt, ellipseFun);
        if (!isOk) {
            printf("Warning: ellipse fitting failed\n");
            continue;
        }
        
		int num_inlier = 0;
		vcl_vector<vgl_point_2d<double>>  pt_inlier;
		for(int j = 0; j<labelPt.size(); ++j)
		{
            for(int k = 0; k<labelPt[j].size(); ++k)
			{
				double x = labelPt[j][k].x();
				double y = labelPt[j][k].y();
                
				double dis = distancePoint2ellipse(x, y, ellipseFun);
				//count the number of points with in W[3] pixel of the ellipse
				if(dis < threshold)
				{
					num_inlier++;
					pt_inlier.push_back(vgl_point_2d<double>(x, y));
				}
			}
		}
		if(num_inlier > num_max)
		{
			num_max = num_inlier;
			arc_pt_max = pt_inlier;
            
			//re computer iterator numbers
			double with_in_ratio = 1.0 * num_max/totalNum;
			int refinedIterNum = log(fail_ratio)/(log(1.0- pow(with_in_ratio, 5)));
            if (refinedIterNum < iterNum) {
                iterNum = refinedIterNum;
            }
			vcl_cout<<"iterator numbers is ----------------------------------------- "<<iterNum<<vcl_endl;
		}
        printf("inliner number is %d %d %d \n", i, num_inlier, num_max);
	}
	
	//show max with_in points
//	vcl_cout<<"number of points with in = "<<arc_pt_max.size()<<vcl_endl;
    fittingPoints = arc_pt_max;
    
	//fit a ellipse with maximum points
	vnl_vector_fixed<double, 5> ellipseMaxNum;
	bool isOk = fitEllipse(arc_pt_max, ellipseMaxNum);
    
    // change tot the format of ellipse
    if (isOk) {
        
     //   void draw_ellipse_majorminor_axis(vil_image_view<vxl_byte> & image, const vnl_vector_fixed<double, 5> & ellipse,
       //                                   const vcl_vector<vxl_byte> & colour);
        ellipseEquation = ellipseMaxNum;
    }
    
    return isOk;
	//draw major and minor axis
//	drawMajorMinorAxis(orgImage, ellipseMaxNum);
}


static bool select5Points(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2,
                          vcl_vector<vgl_point_2d<double> > & arcPts, int maxIter)
{
    // the better way is to randmly suffle
    
    //select two points in first arc
	int id1 = 0;
    int id2 = 0;
    
    int ite = 0;
	do
	{
		id1 = rand()%pts1.size();
		id2 = rand()%pts1.size();
        ite++;
	} while (id1 == id2 && ite < maxIter);
    if (ite == maxIter) {
        printf("Error: can not find sample point in RANSAC.\n");
        return false;
    }
	arcPts.push_back(pts1[id1]);
	arcPts.push_back(pts1[id2]);
    
    //select three points in second arc
    ite = 0;
    int id3 = 0;
	do
	{
		id1 = rand()%pts2.size();
		id2 = rand()%pts2.size();
		id3 = rand()%pts2.size();
        ite++;
	} while ((id1 == id2 ||id1 == id3 || id2 == id3) && ite < maxIter);
    if (ite == maxIter) {
        printf("Error: can not find sample point in RANSAC.\n");
        return false;
    }
	arcPts.push_back(pts2[id1]);
	arcPts.push_back(pts2[id2]);
	arcPts.push_back(pts2[id3]);
	return true;
}


bool VrelPlus::fit_ellipse_RANSAC(const vcl_vector<vgl_point_2d<double> > & pts1,
                                  const vcl_vector<vgl_point_2d<double> > & pts2,
                                  double threshold, double fail_ratio,
                                  vgl_ellipse_2d<double> & ellipse,
                                  vcl_vector<vgl_point_2d<double> > & inliers, int maxIter)
{
    
    int num_max = 0;
	vcl_vector<vgl_point_2d<double>>  arc_pt_max;
    
    vcl_vector<vgl_point_2d<double> > allPts;
    allPts.insert(allPts.begin(), pts1.begin(), pts1.end());
    allPts.insert(allPts.begin(), pts2.begin(), pts2.end());
    
	//total edge numbers
	int iterNum = maxIter;
	for(int i = 0; i<iterNum; ++i)
	{
		//select three points from one arc and two from other
		vcl_vector<vgl_point_2d<double> > arcPt;
		bool isSelected = select5Points(pts1, pts2, arcPt, 500);
        if (!isSelected) {
            continue;
        }
        
        //fit a ellipse
		vnl_vector_fixed<double, 5> ellipseFun;
		bool isOk = fitEllipse(arcPt, ellipseFun);
        if (!isOk) {
            printf("Warning: ellipse fitting failed\n");
            continue;
        }        
		
		vcl_vector<vgl_point_2d<double>>  pt_inlier;
		for(int j = 0; j<allPts.size(); ++j)
		{
            double x = allPts[j].x();
            double y = allPts[j].y();
            
            double dis = distancePoint2ellipse(x, y, ellipseFun);
            //count the number of points with in W[3] pixel of the ellipse
            if(dis < threshold) {
                pt_inlier.push_back(vgl_point_2d<double>(x, y));
            }
		}
		if(pt_inlier.size() > num_max)
		{
			num_max = (int)pt_inlier.size();
			arc_pt_max = pt_inlier;
            
			//re computer iterator numbers
			double with_in_ratio = 1.0 * num_max/allPts.size();
			int refinedIterNum = log(fail_ratio)/(log(1.0- pow(with_in_ratio, 5)));
            if (refinedIterNum < iterNum) {
                iterNum = refinedIterNum;
            }
		}
	}
	
	//show max with_in points
//	vcl_cout<<"number of points with in = "<<arc_pt_max.size()<<vcl_endl;
    inliers = arc_pt_max;
    
	//fit a ellipse with maximum points
    if (inliers.size() >= 8) {
        ellipse = vgl_fit_ellipse_2d_DLT(inliers);
        return true;
    }
    return false;
}





