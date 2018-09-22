#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <curl/curl.h>
#include <string>
#include <exception>

using namespace std;

int flag = 0;
string result;

int writer(char *data, size_t size, size_t nmemb, string *writerData)
{
    if (writerData == NULL)
    {
        return -1;
    }
    int len = size*nmemb;
    writerData->append(data, len);

    return len;
}

int parseJsonRensonse(string input)
{
    Json::Value root;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(input, root);

    if(!parsingSuccessful)
    {
        cout << "!!! Failed to parse the response data" <<endl;
        return 1;
    }
    const Json::Value code = root["code"];
    const Json::Value text = root["text"];
    result = text.asString();
    flag = 1;
    cout << "response code:" << code << endl;
    cout << "response text:" << result << endl;
}

int HttpPoseRequest(string input)
{
    string buffer;

    std::string strJson = "{";
    strJson += "\"key\" : \"25bc3a6a8b784533bb82656afa0f40e1\",";
    strJson += "\"info\" : ";
    strJson += "\"";
    strJson += input;
    strJson += "\"";
    strJson += "}";

    cout<< "post json string:" << strJson << endl;
    try
    {
        CURL *pCurl = NULL;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_ALL);

        //get a curl handle
        pCurl = curl_easy_init();
        if(NULL != pCurl)
        {
            //set url timeout
            curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 8);

            curl_easy_setopt(pCurl, CURLOPT_URL, "http://www.tuling123.com/openapi/api");

            curl_slist *plist = curl_slist_append(NULL,"Content-Type:application/json; charset=UTF-8");
            curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, plist);

            curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, strJson.c_str());

            curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, writer);
            curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, &buffer);

            res = curl_easy_perform(pCurl);

            if(res != CURLE_OK)
            {
                printf("curl_easy_perform() failed:%s\n", curl_easy_strerror(res));
            }
            curl_easy_cleanup(pCurl);
        }
        curl_global_cleanup();
    }
    catch (std::exception &ex)
    {
        printf("!!! curl exception %s.\n", ex.what());
    }

    if(buffer.empty())
    {
        cout << "!!! ERROR The TuLing server response NULL" <<endl;
    }
    else
    {
        parseJsonRensonse(buffer);
    }
}

void nlpCallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout<<"your question is:"<< msg->data << std::endl;
    HttpPoseRequest(msg->data);
}

int main(int argc, char* argv[])
{
        ros::init(argc, argv, "tuling_nlu_node");
        ros::NodeHandle ndHandle;

        ros::Subscriber sub = ndHandle.subscribe("/voice/tuling_nlu_topic", 10, nlpCallback);
        ros::Publisher pub = ndHandle.advertise<std_msgs::String>("/voice/xf_tts_topic", 10);
        ros::Rate loop_rate(10);

        while(ros::ok())
        {
            if(flag)
            {
                std_msgs::String msg;
                msg.data = result;
                pub.publish(msg);
                flag = 0;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }

        return 0;
}

