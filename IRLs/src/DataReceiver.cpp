#include "Communicator.h"

DataReceiver::DataReceiver(int port, std::string tag):
pub_tag(tag)
{
    ready2connect = true;

    socket_desc = socket(AF_INET, SOCK_STREAM, 0);
    if(socket_desc == -1)
    {
        std::cout << "cannot create socket" << std::endl;
        ready2connect = false;
    }

    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    if(bind(socket_desc, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        std::cout << "bind failed" << std::endl;
        ready2connect = false;
    }

    BufferSize = TCP_BUFFER_SIZE;
    DataReceived = new unsigned char[BufferSize];

    BytesReceived = 0;
    sleeping_time = 1e-6;

    if(ready2connect)
    {
        tf_pub = nh.advertise<tf2_msgs::TFMessage>("tf", 1000);
        str_pub = nh.advertise<std_msgs::String>("irl_trainer_sub", 1000);
        init_client = nh.serviceClient<huroco_right_arm::rightInit>("huroco_right_arm/right_init");
        cartesian_client = nh.serviceClient<huroco_right_arm::rightCartesian>("huroco_right_arm/right_cartesian");
        BufferedRecv();
    }
}

DataReceiver::~DataReceiver()
{
    delete[] DataReceived;
}

unsigned int DataReceiver::MessagePublishDemo(const unsigned char* Data)
{
    static unsigned char* buffer = new unsigned char[BufferSize];

    unsigned int Consumed = 0;
    unsigned int Previous = 0;
    
    for(int i=0; i<BytesReceived; i++)
    {
        if(Data[i] == '\n')
        {
            memmove(buffer, Data+Previous, i-Previous);
            buffer[i-Previous] = 0;

            std::string str_data(reinterpret_cast<char *>(buffer));

            Document doc;
            doc.Parse(str_data.c_str());

            int reset = doc["reset"].GetInt();
            if(reset == 1)
            {
                huroco_right_arm::rightInit initSrv;
                init_client.call(initSrv);
                sleep(2);
            }

            Value locObj;
            locObj = doc["location"];
            Value rotObj;
            rotObj = doc["rotation"];

            double locX = locObj["X"].GetDouble();
            double locY = locObj["Y"].GetDouble();
            double locZ = locObj["Z"].GetDouble();

            double rotX = rotObj["X"].GetDouble();
            double rotY = rotObj["Y"].GetDouble();
            double rotZ = rotObj["Z"].GetDouble();
            double rotW = rotObj["W"].GetDouble();

            geometry_msgs::Pose p;
            p.position.x = locX;
            p.position.y = locY;
            p.position.z = locZ;
            p.orientation.x = rotX;
            p.orientation.y = rotY;
            p.orientation.z = rotZ;
            p.orientation.w = rotW;

            std::vector<geometry_msgs::Pose> poseVector = {p};

            huroco_right_arm::rightCartesian cartesianSrv;
            cartesianSrv.request.waypoints = poseVector;

            if(cartesian_client.call(cartesianSrv))
            {
                std::cout << "planning result: " << cartesianSrv.response.status << std::endl;
                if(cartesianSrv.response.status == 0)
                {
                    tf2_msgs::TFMessage msg;
                    tf_pub.publish(msg);
                }
                else if(cartesianSrv.response.status > 0)
                {
                    tf2_msgs::TFMessage msg;
                    geometry_msgs::TransformStamped tmpTF;
                    msg.transforms.push_back(tmpTF);
                    tf_pub.publish(msg);
                }
            }
            else ROS_ERROR("failed to call service right_arm_server");

            Consumed = i+1;
            Previous = i+1;
        }
    }

    return Consumed;
}

unsigned int DataReceiver::MessagePublishTrain(const unsigned char* Data)
{
    static unsigned char* buffer = new unsigned char[BufferSize];

    unsigned int Consumed = 0;
    unsigned int Previous = 0;

    for(int i=0; i<BytesReceived; i++)
    {
        if(Data[i] == '\n')
        {
            memmove(buffer, Data+Previous, i-Previous);
            buffer[i-Previous] = 0;

            std::string str_data(reinterpret_cast<char *>(buffer));

            std_msgs::String tmpStr;
            tmpStr.data = str_data;
            str_pub.publish(tmpStr);

            Consumed = i+1;
            Previous = i+1;
        }
    }

    return Consumed;
}

void DataReceiver::BufferedRecv()
{
    listen(socket_desc, 3);
    int len_addrin = sizeof(struct sockaddr_in);

    client_sock = accept(socket_desc, (struct sockaddr*)&client, (socklen_t*)&len_addrin);
    if(client_sock < 0)
    {
        std::cout << "accept failed" << std::endl;
        return;
    }

    unsigned int read_size;
    while(ros::ok())
    {
        read_size = recv(client_sock, DataReceived+BytesReceived, BufferSize-BytesReceived, 0);
        if(read_size == 0) continue;
        else if(read_size == -1)
        {
            std::cout << "recv failed" << std::endl;
            break;
        }

        BytesReceived += read_size;
        unsigned int Consumed;
        if(pub_tag == "demo") Consumed = MessagePublishDemo(DataReceived);
        else if(pub_tag == "train") Consumed = MessagePublishTrain(DataReceived);
        if(Consumed > 0)
        {
            memmove(DataReceived, DataReceived+Consumed, BytesReceived-Consumed);
            BytesReceived -= Consumed;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DataReceiver");
    if(argc != 3)
    {
        std::cout << "number of arguments is not correct !" << std::endl;
        return -1;
    }
    int port = std::stoi(std::string(argv[1]));
    std::string tag = argv[2];

    DataReceiver dr(port, tag);

    ros::spin();

    return 0;
}
