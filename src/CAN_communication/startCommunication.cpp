#include "ros/ros.h"
#include <atv_acrosser/killApp.h>
#include <boost/thread/thread.hpp>

void openNewTerminal();
bool notificationKilledProcess(atv_acrosser::killApp::Request &req,
atv_acrosser::killApp::Response &res);
void receiveKillCommunication(int argc, char **argv);

boost::mutex mtxTerminal;
boost::mutex::scoped_lock lock (mtxTerminal);
boost::condition_variable condTerminal;

/* It proves the presence of terminal window that execute the process
communication */
bool existTerminal = false;

/******************************************************************
* @function: openNewTerminal
* Thread opens a new terminal and executes the communication
* program. It also remains waiting status on the condition variable
* to launch again the process communication.
*******************************************************************/
void openNewTerminal()
{
	int statusSystem = 0;

	/*Open the first terminal with communication program*/
	existTerminal = true;
	statusSystem = system("gnome−terminal −x ./communication");
	printf("\nTERMINAL OPENED STATUS: %d", statusSystem);

	/* Infinite while, there will be always a condition variable
	which wait a signal from a killed process.
	When the the condition variable will be awake from a killed process,
	it will open a new terminal and execute the communication
	program and wait again another signal from a killed process */

while(1)
{
	/*Condition variable, wait to be awake after the killed process */
	while(existTerminal == true)condTerminal.wait(lock);

	/*Open a new terminal and execute the communication process */
	statusSystem = system("gnome−terminal −x ./communication");
	printf("\nTERMINAL OPENED STATUS: %d", statusSystem);
	if(statusSystem < 0)
			printf("\n PROBLEM TO OPEN THE NEW WINDOW DURING THE
RESTARTING OF THE SOFTWARE communication");
	}
}
/******************************************************************
* @function: receiveKillCommunication
* Thread waits the communication with communication process via
* ROS service in case the process communication needs to
* terminate. When receive the notice from the service the
* function notificationKilledProcess is called.
*******************************************************************/
void receiveKillCommunication(int argc, char **argv)
{
	ros::init(argc, argv, "");
	 ros::NodeHandle n;
	//Here the service called "restartCommunication" is created and
	//advertised over ROS.
	ros::ServiceServer service = n.advertiseService
	("restartCommunication", notificationKilledProcess);
	 ros::spin();
}
/******************************************************************
* @function: notificationKilledProcess
* This function has called each time that ROS service answers from
* the communication creating * a syncronization with it.
* The function will change in false the value of the variable
* existTerminal and wake up the * condition variable condTerminal
* with the scope of open a new terminal and execute the process
* communication.
********************************************************************/
bool notificationKilledProcess(atv_acrosser::killApp::Request &req,
atv_acrosser::killApp::Response &res)
{
	 ROS_INFO("PID KILLED %ld", (long int)req.pid2Kill);
	/* set to false the variable existTerminal, it means there aren’t
	open terminal with running communication */
	existTerminal = false;
	/* wake up the condition variable condTerminal */
	 condTerminal.notify_one();
	return true;
}

int main(int argc, char **argv)
{
boost::thread openNewTerminal_Thread(&openNewTerminal);
boost::thread receiveKillCommunication_Thread(
&receiveKillCommunication, argc, argv);

openNewTerminal_Thread.join();
receiveKillCommunication_Thread.join();

return 0;
}
