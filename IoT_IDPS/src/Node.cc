#include <string.h>
#include <omnetpp.h>
#include <stdio.h>
#include "iot_m.h"
#include <vector>
#include <cmath>

using namespace omnetpp;


class Node : public cSimpleModule
{
    ///////////////////------------------------------------------- Declaration -------------------------------------------///////////////////
  protected:

    virtual void generateMessage();
    virtual void refreshDisplay() const override;
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void generateAttack();
    virtual void generateIDPS();


  private:

      cMessage * timer;                           //This is a normal message called timer. if the handle message received this message it will know that its a self message (we use it as a self message to differentiate between normal message)
      cMessage *one_sec_timer_self_msg =nullptr;   //Message that is created to work as a self message to calculate values every 1 sec
      IotMsg *packetMsg = nullptr;                 //Variable named packetmsg and its type is IoTmsg that is generated from file iot.msg. //this file contains different attribute that can be changed and then the file is build to create m.cc and m.h (we created this type)

      int MAC;                              //Variable mac that is linked to the mac value in .ned by the par() method. this is to get the device ID
      long numSent;                             //We declare variables to watch them through the simulation (sent/received/total power/time)
      long numReceived;
      cHistogram no_packets;                    //cHistogram and cOuterVector are used to record data that can be shown in the result file (here we
      cOutVector no_packets_V;                 //record the number of received/sent packets
      double idlePower;                         //Power parameters used from other resaerch
      double TxPower;
      double RxPower;
      double TotalPower;
      cHistogram S_power_cons;                //The S in s_.. stands for statistics. These are also used to record data
      cOutVector S_power_cons_V;
      double calculate_time;                 //To calculate the simulation time and store it in the result file to analyze data based on time



      cMessage *time_to_attack =nullptr;  //Message that is created to work as a self message for attack nods to start the attack
      bool attack1;                       //Represent the type of attack
      bool attack2;
      int attackerTargetGate;
      int attackerTargetID;

      cMessage * learning_timer = nullptr;
      cMessage * time_to_IDPS = nullptr;
      cMessage * IDPS_check_timer = nullptr;
      bool isIDPS;
      bool there_is_attack1 =false;
      std::set<int> blockedDevices;     //Variable to store blocked Device by the IDPS
      int numConnectedDevices;
      int *connectedDeviceIds;
      int *connectedDevicePackets;
      double *normalTraffic_compare;
      int time_to_learn;
      int no_iteration;
      int phases_to_learn;   //this start as 0 and it is added by 1 every time the time_to_learn work (mainly to keep track for the 2d array)
      std::vector<std::vector<int>> IDS_2d_array;
      double *power_stdDev;
      double power_per_iteration;
      double prev_power_per_iteration;
      double normalPower_compare;

      int idps_check_counte;  //this is a counter that increases when every 120 sec happen (each 120 = 1 iteration) to know how many iteration is there
      int idps_attack_counter; //this increases by 1 every 120 if there is an attack (to know how many attacks)

};

///////////////////------------------------------------------- End of Declaration -------------------------------------------///////////////////

Define_Module(Node);                        //Defining the simple module where the this code will excute

void Node::refreshDisplay() const
{
    char buf[40];
    sprintf(buf, "R: %ld S: %ld", numReceived, numSent);
    this->getParentModule()->getDisplayString().setTagArg("t", 0, buf);
}


void Node::initialize()
{
    int componentId = this->getParentModule()->getId();
    par("MAC").setIntValue(componentId);
    MAC = par("MAC");
    EV << "Insialize  ----------------- " << " Id: "  << MAC<< " now \n" ;
    numSent = 0;                                        //We initiate the the sent and received to 0
    numReceived = 0;
    WATCH(numSent);                                    //WATCH function keeps an eye of the variable when the simulation is runing
    WATCH(numReceived);
    idlePower = par("idle");                          //We give the values of the power parameters from the NED file using the par function
    TxPower = par("Tx");
    RxPower = par("Rx");
    TotalPower=idlePower;                            //The total power starts with the idle power as the device didn't send or receive anything yet.
    WATCH(TotalPower);
    calculate_time = simTime().dbl();                //We register the current simulation time and convert it to double
    WATCH(calculate_time);
    no_packets.setName("no_packets");
    no_packets_V.setName("no_packets");
    S_power_cons.setName("power");
    S_power_cons_V.setName("power");
    power_per_iteration=0;
    prev_power_per_iteration=0;

    attack1=par("attack1");                   //We give the boolean values if its an attacker or IDPS from the NED file and it is assigned in the ini
    attack2=par("attack2");
    attackerTargetGate = -1;
    attackerTargetID = -1;



// ------------------------------------------------------------------- Generate Message------------------------
// ------------------------------------------------------------------- Generate Message------------------------


    int random_timer = intuniform(1, 30)/10;
    timer = new cMessage();
    scheduleAt(simTime() +random_timer, timer);
    one_sec_timer_self_msg = new cMessage();
    scheduleAt(simTime()+1.0, one_sec_timer_self_msg);               // to schedule a self message after 1 sec to record values after every sec
// ------------------------------------------------------------------- Generate Message------------------------
// ------------------------------------------------------------------- Generate Message------------------------


// ------------------------------------------------------------------- IDS------------------------
// ------------------------------------------------------------------- IDS------------------------

    isIDPS=par("isIDPS");
    time_to_learn=120;
    no_iteration = 50;

    numConnectedDevices = gateSize("gate$o");      //this to get the number of connected gates to the node
    connectedDeviceIds = new int[numConnectedDevices];        //we create an array to store the ids of the connected gates
    connectedDevicePackets = new int[numConnectedDevices];
    IDS_2d_array.resize(no_iteration, std::vector<int>(numConnectedDevices));
    power_stdDev= new double [no_iteration];
    for (int i = 0; i < no_iteration; i++) {       //we give it initial 0 values to not get errors
        power_stdDev[i] = 0;
    }

    phases_to_learn=0;


    for (int i = 0; i < numConnectedDevices; i++) { //here is a loop to store the ids of the connected gates in the array
            cModule *connectedModule = gate("gate$o",i)->getPathEndGate()->getOwnerModule();
            connectedDeviceIds[i] = connectedModule->getParentModule()->getId();
            connectedDevicePackets[i] = 0;
        }

    learning_timer = new cMessage("learning_timer");
    scheduleAt(simTime() +time_to_learn, learning_timer);

    if(isIDPS==true){                                           //GenerateIDPS and time_to_IDPS will only work if the isIDPS is true
        normalPower_compare =0;
        idps_check_counte = 0;
        idps_attack_counter=0;
        normalTraffic_compare = new double[numConnectedDevices];
        for (int i = 0; i < numConnectedDevices; i++) {       //we give it initial 0 values to not get errors
            normalTraffic_compare[i] = 0;
        }
        time_to_IDPS = new cMessage("time_to_IDPS");
        scheduleAt(simTime()+(time_to_learn*no_iteration)+1, time_to_IDPS);
        IDPS_check_timer= new cMessage("IDPS_check_timer");
        scheduleAt(simTime()+(time_to_learn*no_iteration)+time_to_learn-1, IDPS_check_timer);

    }

// ------------------------------------------------------------------- IDS------------------------
// ------------------------------------------------------------------- IDS------------------------




// ------------------------------------------------------------------- Attack------------------------
// ------------------------------------------------------------------- Attack------------------------
                                 //If the device is the attacker, send a self message after 10 sec so the attacks start after 10 sec.
    if (attack1==true) {
        time_to_attack = new cMessage("time_to_attack");
        scheduleAt(simTime()+(time_to_learn*no_iteration)+time_to_learn, time_to_attack);
        attack2=false;
        isIDPS=false;
        generateAttack();
    }
    else if (attack2==true){
        time_to_attack = new cMessage("time_to_attack");
        scheduleAt(simTime()+(time_to_learn*no_iteration)+time_to_learn, time_to_attack);
        attack1=false;
        isIDPS=false;
        generateAttack();

        }
// ------------------------------------------------------------------- Attack------------------------
// ------------------------------------------------------------------- Attack------------------------
}

void Node::handleMessage(cMessage *msg)
{


    if (msg == timer) {
        int random_timer = intuniform(10, 30)/10;
        generateMessage();
        numSent++;
        TotalPower= TotalPower+TxPower;
        scheduleAt(simTime() + random_timer, timer);
    }
    else if (msg == one_sec_timer_self_msg){
        no_packets.collect(numReceived);   //we put numReceived because the number of packet sent = the number of packet received which is the number of packets
        no_packets_V.record(numReceived);           //collect and record are methods to record the variables to the analysis and results
        S_power_cons.collect(TotalPower);
        S_power_cons_V.record(TotalPower);
        scheduleAt(simTime()+1.0, one_sec_timer_self_msg);
    }
    else if (msg == learning_timer) {
        scheduleAt(simTime() + time_to_learn, learning_timer);
        power_per_iteration=TotalPower-prev_power_per_iteration;
        prev_power_per_iteration=TotalPower;

        if (phases_to_learn < no_iteration) {
          for (int i = 0; i < numConnectedDevices; ++i) {
            IDS_2d_array[phases_to_learn][i] = connectedDevicePackets[i];
          }
          power_stdDev[phases_to_learn]=power_per_iteration;
          phases_to_learn++;
        }

        EV << "Power per itration :" << power_per_iteration<<"\n";
        for (int i = 0; i < numConnectedDevices; ++i) {
             EV << connectedDevicePackets[i]<<" ,";
         }
         EV << endl;


         for (int i = 0; i < numConnectedDevices; ++i) {        //Reset the number of packets recevied
           connectedDevicePackets[i] = 0;
         }

/*         EV << "just to print (Power) \n";                just to print every (learn time)
         for (int i = 0; i <phases_to_learn ; i++) {
             EV << power_stdDev[i]<<" ,";
         }
         EV << endl;

*/

    }
    else if(msg==time_to_attack){
        if(attack1==true){
            this->getParentModule()->getDisplayString().setTagArg("i", 1, "red"); // Change the attacket node color to red
            generateAttack();
            numSent++;
            this->getParentModule()->bubble("I am attacking");
            scheduleAt(simTime()+0.5, time_to_attack);          //change the attack time here
        }
        else if(attack2==true){
            this->getParentModule()->getDisplayString().setTagArg("i", 1, "crimson"); //Change the attacket node color to crimson
            generateAttack();
            numSent++;
            this->getParentModule()->bubble("I am attacking 2");
            int random_timer = time_to_learn/2;
            scheduleAt(simTime()+random_timer, time_to_attack);          //change the attack time here
        }
    }
    else if(msg==time_to_IDPS){
        this->getParentModule()->bubble("Working IDPS");
        generateIDPS();
    }
    else if(msg==IDPS_check_timer){
//tell other nodes  //change the color of the node to green if it detected an attack // compare normal traffic to others
        EV << "Number of itration" <<idps_check_counte<<"\n";
        idps_check_counte++;
        EV << "Number of attacks" <<idps_attack_counter<<"\n";
        EV << "Power per itration :" << power_per_iteration<<" comapre to: " <<normalPower_compare<< endl;

        EV << "Array of packets to compare \n";
        for (int i = 0; i < numConnectedDevices; ++i) {
             EV << connectedDevicePackets[i]<<" ,";
         }
         EV << endl;

         EV << "prev_total power :" <<prev_power_per_iteration<<"\n";

         for (int i = 0; i < numConnectedDevices; ++i) {
           if(connectedDevicePackets[i] > normalTraffic_compare[i]){
               there_is_attack1 =true;
               EV << "Number of attacks" <<idps_attack_counter<<"\n";
               idps_attack_counter++;
              // blockedDevices.insert(connectedDeviceIds[i]);

               EV << "There is an attack----." << endl;
               EV << "ID: "<<connectedDeviceIds[i] << " is the attacker" << endl;       //This is the id of the attacker
               EV << connectedDevicePackets[i] << " is the new traffic " << normalTraffic_compare[i]<< " is recorded compare"  << endl;
           }
         }

         if(there_is_attack1==true){
             this->getParentModule()->getDisplayString().setTagArg("i", 1, "blue");
             EV << "Blocked Devices: ";
                               for (int deviceId : blockedDevices) {
                                   EV << deviceId << " ";
                               }
                               EV << endl;

         }

         else if(power_per_iteration>normalPower_compare){
             this->getParentModule()->getDisplayString().setTagArg("i", 1, "dodgerblue");
             EV << "There is an attack----." << endl;
         }

        scheduleAt(simTime()+time_to_learn, IDPS_check_timer);
    }

    else {
        calculate_time = simTime().dbl();
        //EV << "Calculate Time varibale " << calculate_time << "]\n";
        IotMsg *imsg = check_and_cast<IotMsg *> (msg);  //we cast the IotMsg to a normal message so the omnet can handle it.
        int msgID = imsg->getSID();           //To check the source of the incoming msg
        if (blockedDevices.find(msgID) != blockedDevices.end()) {
            bubble(("message dropped from " + std::to_string(msgID)).c_str());
              delete msg;
              return;
          }
        bubble("message received");
        numReceived++;

        bool par_attack2 = false;
        for (int i = 0; i < numConnectedDevices; i++) {
          if (connectedDeviceIds[i] == msgID) {                     //Increase the number of packets recieved by 1 from that source
            connectedDevicePackets[i]++;
            EV << "packets recevied from " <<msgID << " is now " <<connectedDevicePackets[i]<<"\n"; //Show the total number of packet recieved from that source
          }
        }

        for (int i = 0; i < numConnectedDevices;i++) {  // here is a loop to store the ids of the connected gates in the array
          if (connectedDeviceIds[i] == msgID) {
            par_attack2 = gate("gate$o", i)->getPathEndGate()->getOwnerModule()->par("attack2");
          }
        }
        if(par_attack2==true && simTime() > (time_to_learn*no_iteration)+time_to_learn && strcmp(msg->getName(), "attack_packet") == 0){
            TotalPower= TotalPower+(RxPower*500);
            EV << "There is attack 2 happening from " << msgID <<"\n";
        }

            TotalPower= TotalPower+RxPower;
            EV << "handlemessage ----------------- " << " The source ID is " <<imsg->getSID() << " The dest Id is " <<imsg->getDID() << " now \n";
           // imsg = nullptr;
            delete imsg;
      }
}


void Node::generateMessage()
{

    int n = gateSize("gate");
    int k = intuniform(0, n-1);
    int src= MAC;
    int des= 0;
    cModule *connectedModule = gate("gate$o",k)->getPathEndGate()->getOwnerModule(); // Get the module pointer connected to the inout gate

    if (connectedModule != nullptr) {  // Check if the module exists
        int connectedModuleID = connectedModule->getParentModule()->getId(); // Retrieve the ID of the module
        des=connectedModuleID;
    }
    else {
        EV << "No module connected to the inout gate." << endl;
    }


    packetMsg = new IotMsg ("packet");      // we use the iot.msg class we built to set the source and destenation. we can assume the iot.msg as a msg header with diff attribute.
    packetMsg->setSID(src);  // the source = the index of the nod
    packetMsg->setDID(des);  //these functions are in the IoTmsg class.
    send(packetMsg, "gate$o", k);

    EV << "genertarmsg ----------------- " << " source: " <<packetMsg->getSID() << "  dest: " <<packetMsg->getDID() << " \n";



}
void Node::generateAttack()
{

    int n = gateSize("gate");    //get gateSize
    int src= MAC;
    int k = intuniform(0, n-1);


if(attackerTargetID==-1){                        //-1 means that there is no target to attack yet. the next setp in the loop picks a random target to attack

    while(attackerTargetID==-1){

        int k = intuniform(0, n-1);
        cModule *connectedModule2 = gate("gate$o",k)->getPathEndGate()->getOwnerModule();



        if (connectedModule2 != nullptr) {

                   bool par_attack1 = connectedModule2->par("attack1");  //we check if the target is an attacker 1st. if yes then we choose another target that is not an attacker
                   bool par_attack2 = connectedModule2->par("attack2");
                   int connectedModuleID = connectedModule2->getParentModule()->getId();
                   if (par_attack1==false && par_attack2==false){
                       attackerTargetID=connectedModuleID;
                       attackerTargetGate=k;
                       EV << "adjacent Node " << connectedModuleID <<" is not an attacker and will be attacked:  " << par_attack1<< endl;
                       break;
                   }
                   else {
                    EV << "Node " << connectedModuleID <<" is the attacker and its type:  " << par_attack1<< endl;
                   }
                               } else {
                                   EV << "No module connected to the inout gate." << endl;
                               }

} }
else{    //this condition happens if a targer is picked. then we just send an attack packet to the target.

    EV << "Attacking" << endl;

     packetMsg = new IotMsg ("attack_packet");           // we use the iot.msg class we built to set the source and destenation. we can assume the iot.msg as a msg header with diff attribute.
     packetMsg->setSID(src);  // the source = the index of the nod and the destination is random from the number of existing nods (vectors)
     packetMsg->setDID(attackerTargetID);  //these functions are in the IoTmsg class.
     send(packetMsg, "gate$o", attackerTargetGate);

     EV << "genertarattack ----------------- " << " source: " <<packetMsg->getSID() << "  dest: " <<packetMsg->getDID() << " \n";
}

}

void Node::generateIDPS(){


     for (size_t i = 0; i < IDS_2d_array.size(); ++i) {
         EV <<"[" <<i << "]";
       for (size_t j = 0; j < IDS_2d_array[i].size(); ++j) {   //this is just to print the 2d array
         EV << IDS_2d_array[i][j] << " ";
       }
       EV << endl;}

     int *one_column_array = new int [IDS_2d_array.size()];   //array varibale to record all the data in a column in order to get the stDev
     EV << " ---------------------- \n";


     for (size_t j = 0; j < IDS_2d_array[0].size(); ++j) {
         for (size_t i = 0; i < IDS_2d_array.size(); ++i) {   //putting all the column from the 2d array in the coloumn array and caclcuate the stDev
             one_column_array[i] = IDS_2d_array[i][j];

         }
         double sum=0;
         double mean=0;
         double sumSquaredDiff = 0;
         for(size_t i =0; i<IDS_2d_array.size();i++){
             sum+= one_column_array[i];
         }
         mean = sum/IDS_2d_array.size();
                                                                   //calculate for the no.packets
         for(size_t i =0; i<IDS_2d_array.size();i++){
             double diff = one_column_array[i] - mean;
             sumSquaredDiff += diff * diff;
         }

         double variance = sumSquaredDiff / IDS_2d_array.size();
         double stdDev =sqrt(variance);

         normalTraffic_compare[j] = mean+(3*stdDev);

         EV << "mean of column is "<< mean << " stdDev is "<< stdDev << " two stand is " << mean+(2*stdDev)  << " 4 stand is "  << mean+(4*stdDev) << "\n";

     }

     double sum2=0;
     double mean2=0;
     double sumSquaredDiff2 = 0;
     for(int i=0; i<no_iteration;i++){
         sum2+= power_stdDev[i];
     }
     mean2 = sum2/no_iteration;
     for(int i=0; i<no_iteration;i++){
         double diff = power_stdDev[i] - mean2;
         sumSquaredDiff2 += diff * diff;
     }
     double variance2 = sumSquaredDiff2 / no_iteration;
     double stdDev2 =sqrt(variance2);

     normalPower_compare=mean2+(3*stdDev2);



     EV << " compare traffic  \n";
          for (int i = 0; i < numConnectedDevices; i++) {
              EV << normalTraffic_compare[i]<<" ,";
          }
          EV << endl;

     EV << " compare power"<< normalPower_compare <<" \n";
     for (int i = 0; i <no_iteration ; i++) {
         EV << power_stdDev[i]<<" ,";
     }
     EV << endl;

     delete one_column_array;
     delete power_stdDev;

}

