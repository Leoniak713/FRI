//  ---------------------- Doxygen info ----------------------
//! \file LWRLoggingExample.cpp
//!
//! \brief
//! Sample application for the class LWRJointPositionController using the
//! data logging functionality
//!
//! \details
//! This simple application feature a sample of how to use the
//! joint position controller \em and how to use the data logging
//! functionality of the KUKA Fast Research Interface
//! for the Light-Weight Robot IV. For details about the actual
//! interface class (i.e., class LWRJointPositionController), please
//! refer to the file LWRJointPositionController.h as well as to the
//! file containing the base controller interface class
//! (i.e., class LWRBaseControllerInterface), LWRBaseControllerInterface.h.
//!
//! \date March 2014
//!
//! \version 1.1
//!
//!	\author Torsten Kroeger, tkr@stanford.edu\n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//! \n
//! \n
//! \copyright Copyright 2014 Stanford University\n
//! \n
//! Licensed under the Apache License, Version 2.0 (the "License");\n
//! you may not use this file except in compliance with the License.\n
//! You may obtain a copy of the License at\n
//! \n
//! http://www.apache.org/licenses/LICENSE-2.0\n
//! \n
//! Unless required by applicable law or agreed to in writing, software\n
//! distributed under the License is distributed on an "AS IS" BASIS,\n
//! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n
//! See the License for the specific language governing permissions and\n
//! limitations under the License.\n
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <LWRJointPositionController.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>


#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS			7
#endif

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif



//*******************************************************************************************
// main()
//
int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;

	int							ResultValue1		=	0
							,	ResultValue2		=	0
							,	i				=	0;

	float						FunctionValue	=	0.0
							,	LoopVariable	=	0.0
							,	JointValuesInRad1[NUMBER_OF_JOINTS]
							,	JointValuesInRad2[NUMBER_OF_JOINTS]
		 					,	InitialJointValuesInRad1[NUMBER_OF_JOINTS]
		 					,	InitialJointValuesInRad2[NUMBER_OF_JOINTS];

	LWRJointPositionController	*Robot1, *Robot2;

	Robot1	=	new LWRJointPositionController("C:\\Users\\FRI\\Documents\\fri_projekty\\lwolinski\\FRILibraryDual\\etc\\Left-FRI-Driver.init");
	Robot2	=	new LWRJointPositionController("C:\\Users\\FRI\\Documents\\fri_projekty\\lwolinski\\FRILibraryDual\\etc\\Right-FRI-Driver.init");

	fprintf(stdout, "RobotJointPositionController object created. Starting the robot...\n");

	ResultValue1	=	Robot1->StartRobot();

	if (ResultValue1 == EOK)
	{
		fprintf(stdout, "Robot1 successfully started.\n");
	}
	else
	{
		fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue1));
	}

	ResultValue2	=	Robot2->StartRobot();

	if (ResultValue2 == EOK)
	{
		fprintf(stdout, "Robot2 successfully started.\n");
	}
	else
	{
		fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue2));
	}

	fprintf(stdout, "Current system 1 state:\n%s\n", Robot1->GetCompleteRobotStateAndInformation());

	fprintf(stdout, "Current system 2 state:\n%s\n", Robot2->GetCompleteRobotStateAndInformation());

	Robot1->GetCommandedJointPositions(InitialJointValuesInRad1);
	Robot2->GetCommandedJointPositions(InitialJointValuesInRad2);

	fprintf(stdout, "Preparing data loggers...\n");
	// Data logging method 1
	Robot1->PrepareLogging("test");
	Robot2->PrepareLogging("test");
	fprintf(stdout, "Data loggers prepared.\n");

	while (LoopVariable < 5.0 * PI) //5.0
	{
		Robot1->WaitForKRCTick();
		Robot2->WaitForKRCTick();

		if (!Robot1->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine 1 is not ready anymore.\n");
			break;
		}

		if (!Robot2->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine 2 is not ready anymore.\n");
			break;
		}

		CycleCounter++;

		if (CycleCounter == 2000)
		{
			// Data logging method 2
			ResultValue1	=	Robot1->StartLogging();

			if (ResultValue1 == EOK)
			{
				// this printf method is real-time capable
				Robot1->printf("Data logging 1 successfully started.\n");
			}
			else
			{
				Robot1->printf("ERROR, cannot start data logging 1.\n");
			}

			ResultValue2	=	Robot2->StartLogging();

			if (ResultValue2 == EOK)
			{
				// this printf method is real-time capable
				Robot2->printf("Data logging 2 successfully started.\n");
			}
			else
			{
				Robot2->printf("ERROR, cannot start data logging 2.\n");
			}
		}

		if (CycleCounter == 12000)
		{
			// Data logging method 3
			ResultValue1	=	Robot1->StopLogging();

			if (ResultValue1 == EOK)
			{
				Robot1->printf("Data logging 1 successfully stopped.\n");
			}
			else
			{
				Robot1->printf("ERROR, cannot stop data logging 1.\n");
			}

			ResultValue2	=	Robot2->StopLogging();

			if (ResultValue2 == EOK)
			{
				Robot2->printf("Data logging 2 successfully stopped.\n");
			}
			else
			{
				Robot2->printf("ERROR, cannot stop data logging 2.\n");
			}
		}

		FunctionValue	=	0.7 * sin(LoopVariable);    //0.3
		FunctionValue	*=	FunctionValue;

		for (i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			JointValuesInRad1[i]	=	InitialJointValuesInRad1[i];
			JointValuesInRad2[i]	=	InitialJointValuesInRad2[i];
		}

		JointValuesInRad1[6]	=	InitialJointValuesInRad1[6]+ FunctionValue;
		JointValuesInRad2[6]	=	InitialJointValuesInRad2[6]+ FunctionValue;


		Robot1->SetCommandedJointPositions(JointValuesInRad1);
		Robot2->SetCommandedJointPositions(JointValuesInRad2);

		LoopVariable	+=	(float)0.001;
	}

	fprintf(stdout, "Writing data file...\n");
	// Data logging method 4
	Robot1->WriteLoggingDataFile();
	Robot2->WriteLoggingDataFile();
	fprintf(stdout, "Data files written.\n");

	fprintf(stdout, "Stopping the robot...\n");
	ResultValue1	=	Robot1->StopRobot();
	ResultValue2	=	Robot2->StopRobot();

	if (ResultValue1 != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot 1...\n");
	}
	else
	{
		fprintf(stdout, "Robot 1 successfully stopped.\n");
	}

	if (ResultValue2 != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot 2...\n");
	}
	else
	{
		fprintf(stdout, "Robot 2 successfully stopped.\n");
	}

	fprintf(stdout, "Deleting the object 1...\n");
	delete Robot1;
	fprintf(stdout, "Object 1 deleted...\n");

	fprintf(stdout, "Deleting the object 2...\n");
	delete Robot2;
	fprintf(stdout, "Object 2 deleted...\n");

	return(EXIT_SUCCESS);
}
