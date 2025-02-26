
package iot_idps;

import inet.networklayer.configurator.contract.IL3NetworkConfigurator;
import inet.node.base.NodeBase;
import inet.node.inet.WirelessHost;
import inet.tutorials.protocol.ClientHost1;
import inet.visualizer.canvas.integrated.IntegratedCanvasVisualizer;


network IoTNetwork
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

        Shost1: Mnode {
            @display("p=50,50;i=block/app2");
        }
        Shost2: Mnode {
            @display("p=250,50;i=block/app2");
        }
        Shost3: Mnode {
            @display("p=50,200;i=block/app2");
        }
        Shost4: Mnode {
            @display("p=250,200;i=block/app2");
        }
        host0: Mnode {
            parameters:
                @display("p=450,50;i=misc/drone");
        }
        host1: Mnode {
            parameters:
                @display("p=450,200;i=misc/drone");
        }
        host2: Mnode {
            parameters:
                @display("p=50,350;i=misc/drone");
        }

        host3: Mnode {
            parameters:
                @display("p=250,350;i=misc/drone");
        }

        host4: Mnode {
            parameters:
                @display("p=450,350;i=misc/drone");
        }



    connections:


//ls=,2,d
        Shost1.g++ <--> Channel <--> Shost2.g++;
        Shost1.g++ <--> Channel <--> Shost3.g++;
        Shost1.g++ <--> Channel <--> Shost4.g++;
        Shost1.g++ <--> Channel { @display("ls=,0"); } <--> host0.g++;
        Shost1.g++ <--> Channel { @display("ls=,0"); } <--> host1.g++;
        Shost1.g++ <--> Channel { @display("ls=,0"); } <--> host2.g++;
        Shost1.g++ <--> Channel { @display("ls=,0"); } <--> host3.g++;
        Shost1.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;

        Shost2.g++ <--> Channel <--> Shost3.g++;
        Shost2.g++ <--> Channel <--> Shost4.g++;
        Shost2.g++ <--> Channel { @display("ls=,0"); } <--> host0.g++;
        Shost2.g++ <--> Channel { @display("ls=,0"); } <--> host1.g++;
        Shost2.g++ <--> Channel { @display("ls=,0"); } <--> host2.g++;
        Shost2.g++ <--> Channel { @display("ls=,0"); } <--> host3.g++;
        Shost2.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;

        Shost3.g++ <--> Channel <--> Shost4.g++;
        Shost3.g++ <--> Channel { @display("ls=,0"); } <--> host0.g++;
        Shost3.g++ <--> Channel { @display("ls=,0"); } <--> host1.g++;
        Shost3.g++ <--> Channel { @display("ls=,0"); } <--> host2.g++;
        Shost3.g++ <--> Channel { @display("ls=,0"); } <--> host3.g++;
        Shost3.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;


        Shost4.g++ <--> Channel { @display("ls=,0"); } <--> host0.g++;
        Shost4.g++ <--> Channel { @display("ls=,0"); } <--> host1.g++;
        Shost4.g++ <--> Channel { @display("ls=,0"); } <--> host2.g++;
        Shost4.g++ <--> Channel { @display("ls=,0"); } <--> host3.g++;
        Shost4.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;


        host0.g++ <--> Channel { @display("ls=,0"); } <--> host1.g++;
        host0.g++ <--> Channel { @display("ls=,0"); } <--> host2.g++;
        host0.g++ <--> Channel { @display("ls=,0"); } <--> host3.g++;
        host0.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;


        host1.g++ <--> Channel { @display("ls=,0"); } <--> host2.g++;
        host1.g++ <--> Channel { @display("ls=,0"); } <--> host3.g++;
        host1.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;

        host2.g++ <--> Channel { @display("ls=,0"); } <--> host3.g++;
        host2.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;

        host3.g++ <--> Channel { @display("ls=,0"); } <--> host4.g++;
}

