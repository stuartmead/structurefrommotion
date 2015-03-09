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

#include <QString>
#include <QDir>
#include <qfileinfo.h>

#include "Workspace/Application/LanguageUtils/streamqstring.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"
#include "Workspace/DataExecution/DataObjects/filefilter.h"


#include "structurefrommotionplugin.h"
#include "filesindirectory.h"


namespace RF
{
    /**
     * \internal
     */
    class FilesInDirectoryImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RF::FilesInDirectoryImpl)

    public:
        FilesInDirectory&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< QString >                           dataDirectory_;
        CSIRO::DataExecution::TypedObject< QString >                           dataFilePatterns_;
        CSIRO::DataExecution::TypedObject< CSIRO::DataExecution::FileFilter >  dataFilters_;
        CSIRO::DataExecution::TypedObject< QStringList >                       dataFiles_;


        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputDirectory_;
        CSIRO::DataExecution::InputArray  inputFilePatterns_;
        CSIRO::DataExecution::InputScalar inputFilters_;
        CSIRO::DataExecution::Output      outputFiles_;


        FilesInDirectoryImpl(FilesInDirectory& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    FilesInDirectoryImpl::FilesInDirectoryImpl(FilesInDirectory& op) :
        op_(op),
        dataDirectory_(),
        dataFilePatterns_(),
        dataFilters_(),
        dataFiles_(),
        inputDirectory_("Directory", dataDirectory_, op_),
        inputFilePatterns_("File patterns", dataFilePatterns_, op_),
        inputFilters_("Filters", dataFilters_, op_),
        outputFiles_("Files", dataFiles_, op_)
    {
    }


    /**
     *
     */
    bool FilesInDirectoryImpl::execute()
    {
        QString&                          directory    = *dataDirectory_;
        CSIRO::DataExecution::FileFilter& filters      = *dataFilters_;
        QStringList&                      files        = *dataFiles_;
        
        files.clear();

        QStringList nameFilters;
        for (int i = 0; i < inputFilePatterns_.size(); ++i)
        {
            nameFilters.push_back(inputFilePatterns_.getInput(i).getDataObject().getRawData<QString>());
        }

        QDir::Filters filt = filters.getAsQDirFilterFlags();
        
        QDir directQ(directory);
        directQ.setFilter(filt);
        directQ.setNameFilters(nameFilters);


        QFileInfoList filesInfo = directQ.entryInfoList();

        for (int j = 0; j < filesInfo.size(); ++j)
        {
            files.push_back(filesInfo.at(j).absoluteFilePath());
        }
        



        return true;
    }


    /**
     *
     */
    FilesInDirectory::FilesInDirectory() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< FilesInDirectory >::getInstance(),
            tr("List files in directory"))
    {
        pImpl_ = new FilesInDirectoryImpl(*this);
    }


    /**
     *
     */
    FilesInDirectory::~FilesInDirectory()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  FilesInDirectory::execute()
    {
        return pImpl_->execute();
    }
}


using namespace RF;
DEFINE_WORKSPACE_OPERATION_FACTORY(FilesInDirectory, 
                                   RF::StructureFromMotionPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("Structure from motion"))

