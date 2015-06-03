#include "csvwriter.hpp"

CSVWriter::CSVWriter()
{
}

void CSVWriter::writeHead(vector<string> head)
{
    string head_str;
    for( std::vector<string>::const_iterator i = head.begin(); i != head.end(); ++i)
    {

         head_str=head_str+*i+delimiter;
    }
    head_str=head_str.substr(0, head_str.size()-1);
    outputFile<<head_str<<endl;

}

void CSVWriter::writeLine(vector<string>data)
{

    string data_str;
    for( std::vector<string>::const_iterator i = data.begin(); i != data.end(); ++i)
    {

         data_str=data_str+*i+delimiter;
    }
    data_str=data_str.substr(0,data_str.size()-1);
    outputFile<<data_str<<endl;
}

void CSVWriter::writeLine(vector<double>data)
{

    string data_str;
    for( std::vector<double>::const_iterator i = data.begin(); i != data.end(); ++i)
    {

        char *str = new char[100] ;
        sprintf(str,"%f",*i);
        data_str=data_str+str+delimiter;
    }
    data_str=data_str.substr(0,data_str.size()-1);
    outputFile<<data_str<<endl;
}

void CSVWriter::openFile(string fname, string sep, int write_head)
{

   outputFile.open(fname.c_str(),ios_base::out);
   delimiter=sep;
}

void CSVWriter::closeFile()
{
    outputFile.close();
}
