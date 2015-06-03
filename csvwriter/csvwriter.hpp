#ifndef CSVWRITER_H
#define CSVWRITER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iterator>
#include <sstream>
#include <alloca.h>



using namespace std;

class CSVWriter
{
private:
    ofstream outputFile;
     string delimiter;

public:
    CSVWriter();
    void writeHead(vector<string>);
    void writeLine(vector<string>);
    void writeLine(vector<double>);
    void openFile(string name,string delimiters,int write_head);
    void closeFile();

};

#endif // CSVWRITER_H
