
package iot_idps;

import inet.mobility.contract.IMobility;


simple Node
{
    parameters:
        @display("i=block/routing");
        
        
        
        double voltage = 3.0;                                    //Voltage value for the power
        double idle = 320e-6*voltage;                            //Power value for the idle case
        double Tx = 19.3e-3*voltage;							//Power trasmiting value
        double Rx = 21.5e-3*voltage;                            //Power receiver value
        int MAC @mutable=0;                                   //This is used as the device ID from getId() @mutable allows a variable to be changed during run time
    
        bool attack1 = default(false);
        bool attack2 = default(false);
        bool isIDPS = default(false);
        
        
        //------------------------------------------- Testing Parameters -------------------------------------------//
        
      
    gates:
        inout gate[];                                         //To declare two way connections for the simple Node
}




        //---------------------------------------- Mnode Module parameters ----------------------------------------//
        
        

module Mnode
{
    parameters:
        @networkNode();
        @display("i=device/pocketpc");
        @statistic[coordX](title="coord X"; source="xCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @statistic[coordY](title="coord Y"; source="yCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @statistic[coordZ](title="coord Z"; source="zCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
    gates:
        inout g[];                                          //To declare two way connections for the Mnode
    submodules:
        mobility: <default("StationaryMobility")> like IMobility {
            @display("p=100,100;is=s");
        }
        node: Node;                                          //Adding the simple Module (Node) to the compund module (Mnode)
         connections:
        for i=0..sizeof(g)-1 {                              //Connecting the gates of the simple and compund Module
            g++ <--> node.gate++;
        }
 
}
