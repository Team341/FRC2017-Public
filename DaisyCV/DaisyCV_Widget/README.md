This widget is meant to be used as an extension in SmartDashboard. <b>ALL OF THE REQUIRED LIBRARIES ARE STORED IN THE DEPENDS FOLDER</b>. A version of the SmartDashboard is also included. This version is guarunteed to work with the extension (there is no money back offer, though).

By default, SmartDashboard.jar is located in (user.home)\wpilib\tools. If you have not run it yet, run it now. This will create the folder
*(user.home)\SmartDashboard. It is recommended to move the SmartDashboard.jar file to this location.

If the file (user.home)\SmartDashboard\extensions does not exist, create it as well as the subfile (user.home)\SmartDashboard\extensions\lib

In the (user.home)\SmartDashboard\extentions folder, put:

	1) The DaisyCV.jar, which has the .class file for the DaisyCV widget. This jar is exported from the source.
		-In eclipse, open the widget project and navigate to File->Export->Java->JAR File
	2) WPICameraExtention.jar
	
In the (user.home)\SmartDashboard\extentions\lib folder, put:

	1) javacpp.jar
	2) javacv.jar
	3) opencv.jar
	4) opencv-windows-yourarchitecture.jar
	8) WPIJavaCV.jar


*(user.home) is your account's user folder typically located in C:\Users\(user.home)

If, in the future, there are updated versions of all these libraries, feel free to update them here in the repository. You might want to keep all these files as is for legacy purposes; they will always work together. Dependancies are scary!