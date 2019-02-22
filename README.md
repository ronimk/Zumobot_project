This repo contains my solutions to the introduction course to Smart Systems and IoT offered by Metropolia UAS to all first year students.

There were three main tasks to be completed in teams of three student in the course - all tasks involving Zumobot
    (https://www.pololu.com/category/169/zumo-robot-for-arduino)

In task 1 we had to program a track-driving robot that starts by pressing a remote control button.
In task 2 we had to program a sumo wrestling roobot.
In task 3 we had to program a maze solver.

I programmed tasks 1 and 3 myself and one of my teammates programmed task 2 (which for that reason is not shown here). We did have problems with managing time since only two in our team were capable of coding the required final tasks. Therefore we decided to divide the work this way instead of both of us being partly involved in all of the final tasks. Since tasks 1 and 2 could easily reuse the same line following logic I had already implemented before, and since we didn't have much time left to work on the project between just the two of us, I volunteered to do the both the first and third tasks.

The scheduling problems shine through in the third (final) task since I never got to implement the breadth-first-search pathfinding algorithm I had designed for solving the maze perfectly. I hardly had time to implement the linked-list datastructure to support the pathfinding algorithm, much less test it for anything more than basic functionality. Therefore I am not completely sure if it works without memory leakages.

Of course the linked list DS is not used in the program in any way, so this is just a theoretical issue now.

Another problem was that we weren't able to completely familiarize ourselves with the Zumobot during the course, and this shows in some parts of the code.

We also could have structured the code better and separate some of the code in this file into their own library/libraries...

Still, overall the course taught me quite a bit and I am very satisfied with my simple yet elegant solution for deciding where to drive next along a line (turn, straight or backwards). Also, figuring out how to utilize a 2d array optimally for the maze solver task was really fun and instructional, and I'm very satisfied with the idea I came up with,  using a tiny bit of extra info in the maze array to both avoid complicating the search for the next destination node when the robot is on an edge of the maze and to aid in printing the maze to the MQTT server.

The main program contains the final unaltered solutions (except for some cleaning in the comment sections). Thus it has much room for improvement!

The project uses MQTT, FreeRTOS and Zumobot libraries... To make the code work, insert the proper MQTT info in the zumo_config.h file.
