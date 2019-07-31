#include "Communicator.h"

DataSender::DataSender(int port, std::string ip_addr, std::string tag):
sub_tag(tag)
{
    cnt = 0;
    sock_connected = true;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1)
    {
        std::cout << "cannot create socket" << std::endl;
        return;
    }

    server.sin_addr.s_addr = inet_addr(ip_addr.c_str());
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        std::cout << "connect failed" << std::endl;
        sock_connected = false;
    }

    if(sub_tag == "demo") tf_sub = nh.subscribe("tf", 1000, &DataSender::OnDataRecvDemo, this);
    else if(sub_tag == "train") str_sub = nh.subscribe("irl_trainer_pub", 1000, &DataSender::OnDataRecvTrain, this);

    if(sock_connected) ProcDataRecv();
}

DataSender::~DataSender()
{}

void DataSender::OnDataRecvDemo(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    std::string str_data;
    Document doc;
    Document::AllocatorType &allocator = doc.GetAllocator();
    doc.SetObject();
    Value animArray(kArrayType);
    int execResult;

    if(msg->transforms.size() == 0)
    {
        execResult = 0;
        std::cout << "execution failed" << std::endl;
    }
    else if(msg->transforms.size() == 1)
    {
        execResult = 1;
        std::cout << "execution succeeded" << std::endl;
    }
    else execResult = -1;

    for(int i=0; i<msg->transforms.size(); i++)
    {
        Value thisAnim(kObjectType);
        Value tmpStr(msg->transforms[i].child_frame_id.c_str(), allocator);
        thisAnim.AddMember("MeshName", tmpStr, allocator);

        Value locationObj(kObjectType);
        locationObj.AddMember("X", msg->transforms[i].transform.translation.x, allocator);
        locationObj.AddMember("Y", msg->transforms[i].transform.translation.y, allocator);
        locationObj.AddMember("Z", msg->transforms[i].transform.translation.z, allocator);
        thisAnim.AddMember("Loc", locationObj, allocator);

        Value rotationObj(kObjectType);
        rotationObj.AddMember("X", msg->transforms[i].transform.rotation.x, allocator);
        rotationObj.AddMember("Y", msg->transforms[i].transform.rotation.y, allocator);
        rotationObj.AddMember("Z", msg->transforms[i].transform.rotation.z, allocator);
        rotationObj.AddMember("W", msg->transforms[i].transform.rotation.w, allocator);
        thisAnim.AddMember("Rot", rotationObj, allocator);

        animArray.PushBack(thisAnim, allocator);
    }

    doc.AddMember("ExecResult", execResult, allocator);
    doc.AddMember("Anim", animArray, allocator);
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    doc.Accept(writer);

    std::string str_json = buffer.GetString();
    str_data = str_json + "\n";

    if(send(sock, str_data.c_str(), strlen(str_data.c_str()), 0) < 0)
    {
        std::cout << "send failed" << std::endl;
        return;
    }
    cnt = (cnt+1) % 100;
}

void DataSender::OnDataRecvTrain(const std_msgs::String::ConstPtr& msg)
{
    std::string str_data = msg->data;
    str_data += "\n";
    std::cout << str_data << std::endl;

    if(send(sock, str_data.c_str(), strlen(str_data.c_str()), 0) < 0)
    {
        std::cout << "send failed" << std::endl;
        return;
    }
}

void DataSender::ProcDataRecv()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DataSender");

    if(argc != 4)
    {
        std::cout << "number of arguments is not correct !" << std::endl;
        return -1;
    }
    int port = std::stoi(std::string(argv[1]));
    std::string ip_addr = argv[2];
    std::string tag = argv[3];

    DataSender ds(port, ip_addr, tag);

    return 0;
}
