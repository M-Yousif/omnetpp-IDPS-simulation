
package iot_idps;

import inet.networklayer.configurator.contract.IL3NetworkConfigurator;
import inet.node.base.NodeBase;
import inet.node.inet.WirelessHost;
import inet.tutorials.protocol.ClientHost1;
import inet.visualizer.canvas.integrated.IntegratedCanvasVisualizer;


network IoTNetwork2
{
    parameters:
        //------------------------------------------- Network Parameters -------------------------------------------// 

        int numHosts;                                                          //number of devices in the network
        double packet_size = default(1500);                                   // maxiumum packter size

        //------------------------------------------- Extra Parameters -------------------------------------------//

        bool hasVisualizer = default(false);

    types:
        //--------------------------------------------- Connections ---------------------------------------------//

        channel Channel extends ned.DelayChannel
        {
            delay = 500ms;                                                   // Delay between connections
        }
    submodules:
        //------------------------------------------- Network devices -------------------------------------------//
        visualizer: IntegratedCanvasVisualizer if hasVisualizer {
            parameters:
                @display("p=540,340");
        }
        Nhost [numHosts]: Mnode {
          @display("i=block/app2");
        }
     

 

    connections:

 for i=0..sizeof(Nhost)-1, for j=i+1..sizeof(Nhost)-1 {
            Nhost[i].g++ <--> Channel { @display("ls=,0"); } <-->  Nhost[j].g++;
        }
       
//for i=1..sizeof(Nhost)-1 {
//            Nhost[0].g++ <--> Channel <--> Nhost[i].g++;
//        }

      
}
      

