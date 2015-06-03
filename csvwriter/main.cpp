#include <iostream>
#include "csvwriter.h"

using namespace std;

int main()
{
    CSVWriter test_writer;
    vector<string> head;
    head.push_back("name");
    head.push_back("id");
    head.push_back("age");
    vector <string> data1;
    data1.push_back("mahesh");
    data1.push_back("123");
    data1.push_back("33");
    test_writer.openFile("/tmp/test.csv",";",1);
    test_writer.writeHead(head);
    test_writer.writeLine(data1);
    return 0;
}

