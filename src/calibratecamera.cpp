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
#include <qimage.h>
#include <QString>
#include <qvector.h>


#include "Workspace/Application/LanguageUtils/streamqstring.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"


#include "structurefrommotionplugin.h"
#include "calibratecamera.h"
#include "wsmat.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"


namespace RF
{
    using namespace cv;
    /**
     * \internal
     */
    class CalibrateCameraImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RF::CalibrateCameraImpl)

    public:
        CalibrateCamera&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< QImage >           dataChessboardImages_;
        CSIRO::DataExecution::TypedObject< int >              dataBoardWidth_;
        CSIRO::DataExecution::TypedObject< int >              dataBoardHeight_;
        CSIRO::DataExecution::TypedObject< double >           dataBoardScale_;
        CSIRO::DataExecution::TypedObject< WSMat >            dataCalibrationMatrix_;
        CSIRO::DataExecution::TypedObject< QImage >   dataOutImage_;


        // Inputs and outputs
        CSIRO::DataExecution::InputArray  inputChessboardImages_;
        CSIRO::DataExecution::InputScalar inputBoardWidth_;
        CSIRO::DataExecution::InputScalar inputBoardHeight_;
        CSIRO::DataExecution::InputScalar inputBoardScale_;
        CSIRO::DataExecution::Output      outputOutImage_;
        CSIRO::DataExecution::Output      outputCalibrationMatrix_;


        CalibrateCameraImpl(CalibrateCamera& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    CalibrateCameraImpl::CalibrateCameraImpl(CalibrateCamera& op) :
        op_(op),
        dataChessboardImages_(),
        dataBoardWidth_(),
        dataBoardHeight_(),
        dataBoardScale_(1.0),
        dataCalibrationMatrix_(),
        inputChessboardImages_("Chessboard Images", dataChessboardImages_, op_),
        inputBoardWidth_("Board Width", dataBoardWidth_, op_),
        inputBoardHeight_("Board Height", dataBoardHeight_, op_),
        inputBoardScale_("Board Scale", dataBoardScale_, op_),
        outputOutImage_("Out Image", dataOutImage_, op_),
        outputCalibrationMatrix_("Calibration Matrix", dataCalibrationMatrix_, op_)
    {
        inputBoardScale_.setDescription("Size (in units of your choice) of the grids in the chessboard pattern");
    }


    /**
     *
     */

    Mat QImage2Mat(QImage const& src2)
    {
        QImage src = src2.convertToFormat(QImage::Format_RGB32);
        Mat tmp(src.height(), src.width(), CV_8UC4, (uchar*)src.bits(), src.bytesPerLine());
        Mat result; //Deep copy
        cvtColor(tmp, result, CV_BGR2RGB);
        return result;
    }

    bool CalibrateCameraImpl::execute()
    {
        int&             boardWidth        = *dataBoardWidth_;
        int&             boardHeight       = *dataBoardHeight_;
        WSMat& calibrationMatrix = *dataCalibrationMatrix_;
        QImage&  outImage      = *dataOutImage_;

        if (inputChessboardImages_.size() == 0)
        {
            std::cout << QString("ERROR: No images supplied to operation") + "\n";
            return false;
        }
        //Create clean access topinput
        QVector<QImage*> images;
        for (int i = 0; i < inputChessboardImages_.size(); ++i)
        {
            images.push_back(&inputChessboardImages_.getInput(i).getDataObject().getRawData<QImage>());
        }

        int boardTotal = boardWidth*boardHeight;
        Size boardSize = cvSize(boardWidth,boardHeight);


        std::vector<std::vector<Point2f> > imagePoints; //Array of image points
        std::vector<std::vector<Point3f> > objectPoints; //Array of world co-ordinate points

        std::vector<Point3f> worldPoints; //Our actual world points
        double sc = *dataBoardScale_;
        for (int wp = 0; wp < boardTotal; ++wp)
        {
            Point3f loc;
            loc.x = (wp%boardWidth)*sc;
            loc.y = (wp/boardWidth)*sc;
            loc.z = 0.0;
            worldPoints.push_back(loc);
        }
        std::cout << QString("Initialised calibration arrays, beginning image analysis") + "\n";
        int nimages = 0;
        Mat matchView;
        Size imageSz;
        for (int i = 0; i < images.size(); ++i)
        {
            Mat view = QImage2Mat(*images[i]);
            std::vector<Point2f> corners;
            //find chessboard corners
            bool found = findChessboardCorners(view,boardSize,corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
            if (found)
            {
                std::cout << QString("Found correct corners for image %1").arg(i) + "\n";
                Mat grey;
                cvtColor(view, grey, COLOR_BGR2GRAY);
                cornerSubPix(grey,corners,Size(11,11),Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                imagePoints.push_back(corners);//Add the corners to the input array
                objectPoints.push_back(worldPoints);//Add our worldpoint array
                drawChessboardCorners(view,boardSize,corners,found);
                cvtColor(view,view,CV_BGR2RGB);
                QImage qtmp((uchar*) view.data, view.cols, view.rows, view.step, QImage::Format_RGB888);
                QImage qtmp2(qtmp);
                qtmp2.detach();
                outImage = qtmp2;
                imageSz = grey.size();
                ++nimages;
            }
            
        }
        std::cout << QString("Camera detection complete, using %1 images for calibration").arg(nimages) + "\n";
        if (nimages > 1)
        {
        calibrationMatrix.eye(3,3,CV_64F);
        WSMat distortions;
        distortions.zeros(8, 1, CV_64F);
        std::vector<Mat> rotations, translations;
        double er = calibrateCamera(objectPoints,imagePoints, imageSz, calibrationMatrix, distortions, rotations, translations);
        std::cout << QString("Camera calibration complete, error is %1").arg(er) + "\n";
        }
        

        

    
        return true;
    }


    /**
     *
     */
    CalibrateCamera::CalibrateCamera() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< CalibrateCamera >::getInstance(),
            tr("Calibrate Camera"))
    {
        pImpl_ = new CalibrateCameraImpl(*this);
    }


    /**
     *
     */
    CalibrateCamera::~CalibrateCamera()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  CalibrateCamera::execute()
    {
        return pImpl_->execute();
    }
}


using namespace RF;
DEFINE_WORKSPACE_OPERATION_FACTORY(CalibrateCamera, 
                                   RF::StructureFromMotionPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("Structure from motion"))

