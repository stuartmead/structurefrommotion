/*
Copyright (c) 2015, Stuart Mead - Risk Frontiers, Macquarie University
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software without
      specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  For further information, contact:
    Stuart Mead
    Risk Frontiers
    Dept. of Environmental Sciences
    Macquarie University
    North Ryde NSW 2109
*/

#include <cassert>
#include <fstream>


#include <QString>

#include "Workspace/Application/LanguageUtils/streamqstring.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/ocl/ocl.hpp"//ocl


#include "wsmat.h"
#include "structurefrommotionplugin.h"
#include "matchfeatures.h"


namespace RF
{
    /**
     * \internal
     */
    using namespace cv;
    class MatchFeaturesImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RF::MatchFeaturesImpl)

    public:
        MatchFeatures&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< std::vector<cv::KeyPoint> >  dataKeypoints1_;
        CSIRO::DataExecution::TypedObject< RF::WSMat >                  dataDescriptors1_;
        CSIRO::DataExecution::TypedObject< std::vector<cv::KeyPoint> >  dataKeypoints2_;
        CSIRO::DataExecution::TypedObject< RF::WSMat >                  dataDescriptors2_;
        CSIRO::DataExecution::TypedObject< double >                     dataDistanceRatio_;
        CSIRO::DataExecution::TypedObject< bool >                       dataOpenCL_;
        CSIRO::DataExecution::TypedObject< std::vector<cv::DMatch> >    dataGoodMatches_;
        CSIRO::DataExecution::TypedObject< bool >                       dataDrawMatches_;
        CSIRO::DataExecution::TypedObject< QString >                    dataImage1_;
        CSIRO::DataExecution::TypedObject< QString >                    dataImage2_;
        CSIRO::DataExecution::TypedObject< bool >                       dataWriteMatches_;
        CSIRO::DataExecution::TypedObject< QString >                    dataMatchFileLoc_;
        CSIRO::DataExecution::TypedObject< QImage >                     dataImageMatches_;
        CSIRO::DataExecution::TypedObject< QString >                    dataMatchFileString_;


        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputKeypoints1_;
        CSIRO::DataExecution::InputScalar inputDescriptors1_;
        CSIRO::DataExecution::InputScalar inputKeypoints2_;
        CSIRO::DataExecution::InputScalar inputDescriptors2_;
        CSIRO::DataExecution::InputScalar inputDistanceRatio_;
        CSIRO::DataExecution::InputScalar inputOpenCL_;
        CSIRO::DataExecution::Output      outputGoodMatches_;
        CSIRO::DataExecution::InputScalar  inputDrawMatches_;
        CSIRO::DataExecution::InputScalar inputImage1_;
        CSIRO::DataExecution::InputScalar inputImage2_;
        CSIRO::DataExecution::InputScalar inputWriteMatches_;
        CSIRO::DataExecution::Output      outputImageMatches_;
        CSIRO::DataExecution::Output      outputMatchFileString_;


        MatchFeaturesImpl(MatchFeatures& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    MatchFeaturesImpl::MatchFeaturesImpl(MatchFeatures& op) :
        op_(op),
        dataKeypoints1_(),
        dataDescriptors1_(),
        dataKeypoints2_(),
        dataDescriptors2_(),
        dataDistanceRatio_(),
        dataOpenCL_(false),
        dataGoodMatches_(),
        dataDrawMatches_(false),
        dataImage1_(),
        dataImage2_(),
        dataWriteMatches_(true),
        dataImageMatches_(),
        dataMatchFileString_(),
        inputKeypoints1_("Keypoints 1", dataKeypoints1_, op_),
        inputDescriptors1_("Descriptors 1", dataDescriptors1_, op_),
        inputKeypoints2_("Keypoints 2", dataKeypoints2_, op_),
        inputDescriptors2_("Descriptors 2", dataDescriptors2_, op_),
        inputDistanceRatio_("Lowe Distance Ratio", dataDistanceRatio_, op_),
        inputOpenCL_("Use openCL feature matching", dataOpenCL_, op_),
        outputGoodMatches_("Good Matches", dataGoodMatches_, op_),
        inputDrawMatches_("Draw Matches", dataDrawMatches_, op_),
        inputImage1_("Image 1", dataImage1_, op_),
        inputImage2_("Image 2", dataImage2_, op_),
        inputWriteMatches_("Write out matches", dataWriteMatches_, op_),
        outputImageMatches_("Image Matches", dataImageMatches_, op_),
        outputMatchFileString_("Match String", dataMatchFileString_, op_)
    {
        inputWriteMatches_.setDescription("Writes out a QString specifying image matches for use in VSFM");
    }


    /**
     *
     */
    bool MatchFeaturesImpl::execute()
    {
        std::vector<cv::KeyPoint>& keypoints1    = *dataKeypoints1_;
        RF::WSMat&                 descriptors1  = *dataDescriptors1_;
        std::vector<cv::KeyPoint>& keypoints2    = *dataKeypoints2_;
        RF::WSMat&                 descriptors2  = *dataDescriptors2_;
        double&                    distanceRatio = *dataDistanceRatio_;
        std::vector<cv::DMatch>&   goodMatches   = *dataGoodMatches_;
        QString&                   image1        = *dataImage1_;
        QString&                   image2        = *dataImage2_;
        QImage&                    imageMatches  = *dataImageMatches_;
        QString&                   fileStr       = *dataMatchFileString_;
        
        dataGoodMatches_->clear();
        
        std::vector<std::vector<cv::DMatch> > matches;

        if (*dataOpenCL_)
        {
            //std::cout << QString("WARNING: OpenCL is a bit ropey, tends to crash your video card. Don't blame me if you didn't update your drivers.") + "\n";
            ocl::DeviceType dt = ocl::Context::getContext()->getDeviceInfo().deviceType;
            //std::cout << QString("OpenCL device type is %1").arg(dt) + "\n";
            //Run matchings
            ocl::BFMatcher_OCL matcher(NORM_HAMMING2);
            ocl::oclMat ocldesc1, ocldesc2;
            ocldesc1.upload(descriptors1);
            ocldesc2.upload(descriptors2);
            matcher.knnMatch(ocldesc1, ocldesc2, matches,2);
        }
        else
        {
            cv::BFMatcher matcher(NORM_HAMMING2);
            matcher.knnMatch(descriptors1,descriptors2,matches,2);

        }

        //Nearest neighbour filtering - see Frank Lowe's paper
        for (int i = 0; i < matches.size(); ++i)
        {
            if (matches[i][0].distance < distanceRatio * matches[i][1].distance)
            {
                goodMatches.push_back(matches[i][0]);
            }
        }

        //std::cout << QString("Found %1 matches").arg(goodMaiftches.size()) + "\n";

        if (*dataDrawMatches_)
        {
            if (image1.isEmpty() || image2.isEmpty())
            {
                std::cout << QString("WARNING: Draw matches option is on, but image strings are empty. Skipping.") + "\n";
            }
            else
            {
                WSMat outimg;
                cv::Mat img1 = imread(image1.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
                cv::Mat img2 = imread(image2.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
                drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                cvtColor(outimg,outimg,CV_BGR2RGB);
                imageMatches = outimg.convertToQImage();
            }
        }

        if (*dataWriteMatches_ && goodMatches.size() > 0)
        {
            //Matchfile to qstring
            QString ofloc;
            //Image-Match (path 1) (path2) (# matches)
            fileStr = QString("%1 %2 %3\n").arg(image1).arg(image2).arg(goodMatches.size());

            //List of 0 based feature indices in image 1
            for (int i = 0; i < goodMatches.size(); ++i)
            {
                fileStr = fileStr + QString("%1 ").arg(goodMatches[i].queryIdx);
            }
            fileStr.append("\n");
             //List of 0 based feature indices in image 2
            for (int i = 0; i < goodMatches.size(); ++i)
            {
              //  outfile << goodMatches[i].trainIdx << " ";
                fileStr = fileStr + QString("%1 ").arg(goodMatches[i].trainIdx);
            }
            fileStr.append("\n");
        }

        

        return true;
    }


    /**
     *
     */
    MatchFeatures::MatchFeatures() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< MatchFeatures >::getInstance(),
            tr("Match Image Features"))
    {
        pImpl_ = new MatchFeaturesImpl(*this);
    }


    /**
     *
     */
    MatchFeatures::~MatchFeatures()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  MatchFeatures::execute()
    {
        return pImpl_->execute();
    }
}


using namespace RF;
DEFINE_WORKSPACE_OPERATION_FACTORY(MatchFeatures, 
                                   RF::StructureFromMotionPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("Structure from motion"))

