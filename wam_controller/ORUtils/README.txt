Instructions for visualizing in OpenRave

1. Start OpenRave
2. Run exInitializeORScene
	This will load the table and the WAM into the scene. 
	This only needs to be run right after starting Openrave
3. Select the grasp plan to run on by setting grasp_plan_filename in exTestGraspSetup
	Note: The file paths are hardcoded for the machine in the lab
	Note: To run without the tracking system, be sure to comment out the following lines
		exVisualizeGraspPlan: 
			line 27: initTrackingSystem()
				through
			line 41: trackDataRcvd()
		You must also supply your own TWAMTray.  This is the pose of the tray in the WAM's frame
		
4. Run exTestGraspSetup
5. Select the desired grasp when prompted
6. Press [Enter] to clear the scene.  Return to step 4. to run another experiment