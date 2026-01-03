1) Intro  
   1) Basic Goals  
      1) Get excited, or at least interested in programming  
      2) See how it can be useful; put another arrow in your quiver  
   2) What Not To Expect  
      1) Being able to code up a robot in a jiffy that does everything you want  
         1) I can’t.  
      2) Becoming an expert at programming  
      3) Being able to start a project of any kind of scratch  
         1) It’s harder than you think, depending on what it is, and you only need one team member to do it per project so it doesn’t happen very often.  
   3) What you should expect  
      1) Being able to work on an existing project.  
      2) Familiarity with some very common developer tools like Visual Studio Code, git, GitHub.  
      3) A working knowledge of Python; enough to solve robotics problems but also something you could use in your math classes.  
         1) Not just notebooks but you can also program graphing calculators in
         MicroPython like the VEX bots.  
         2) And we have side-projects with microprocessors where MicroPython
         and CircuitPython are great fits. These are low complexity but high
         value things at times.  
         3) Most development, like apps and data engineering, are less
         complicated than robotics.   
      4) Being able to explain how the robot works to a Judge at a competition.  
   4) Common Robot Programming Components and Tasks  
      1) Swerve Drivetrain: Four wheels that can turn and rotate independently.   
         1) Use of kinematics calculations to determine how best to configure those wheels to go where it should  
         2) Encoder and gyro based odometry  
      2) Computer Vision: This is usually done with a different processor (computer) than the actual robot computer. Generally we’ve used Orange Pi’s with PhotonVision.  
         1) AprilTags are used to determine the robots location on the field. We can fuse these readings into the swerve drive odometry to determine where we are on the field as best as possible.  
         2) Object Detection via AI/ML computer vision is also an option depending on the game. Here the robot can locate a game piece in the camera’s view and then use that data to drive toward it.  
      3) Intakes and Game Piece movement  
         1) This is where the robot takes in a game piece and moves it into whatever position the piece should be in to accomplish the next task. Generally shooting or placing the object somewhere.  
         2) This is where things like state machines, or getting the robot to do a series of things like: “run the intake motors at 30% power until the photoeye that indicates we have the piece in place is tripped, but also let it run for 0.04 seconds after the eye is tripped.” come into play.  
      4) Shooting or Placement  
         1) Now everything has to come together. Odometry, maybe more vision, intakes, transitions and shots or placement need to be consistent.  
      5) Controller / Gamepad handling  
         1) This one sounds simple but with feedback from the drive team the controller systems can be made better over time. It’s hard to decide on a perfect button and stick setup (or custom controller\!) before we’ve actually driven the robot. So, it evolves over time.  
      6) Lights/LEDS: Useful for signaling things to the drive team  
         1) It’s a fairly low risk area of development on the robot because it doesn’t involve any moving parts.  
         2) Low risk doesn’t mean low value. If the drive team doesn’t know what the robot is doing or what’s happening inside it, they can’t perform their job well.   
   5) Wiring and Electronics  
      1) This is more Elijah and Ethan’s domain.  
      2) A properly wired robot isn’t just more reliable it’s also safe.  
      3) There’s some serious art to it  
   6) How the lab works  
      1) Shared logins, the real code lives in GitHub. Most sessions will start with pulling a fresh copy of what we last worked on.  
         1) Branches can be used to keep private experiments going over multiple sessions  
         2) Dedicated logins are possible if you want your custom workspace to always be there  
      2) PCs are not very fast or expensive. They’re running Debian Linux, similar to what the actual robot runs.  
      3) The official driver station is a Windows PC because Linux isn’t supported for that role.  
   7) Software Tools  
      1) Visual Studio Code (aka VSCode or just Code)  
         1) 

