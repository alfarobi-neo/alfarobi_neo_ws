#include "model_based_control/csvManager.h"

namespace robotis_op
{
csvManager::csvManager()
{	
}

csvManager::~csvManager()
{	
}

std::vector<std::vector<double>> csvManager::getMatrix(std::string fileName)
{
    ifstream ip(fileName);  
    // std::cout<<fileName<<std::endl;
    if(!ip.is_open()) std::cout << "ERROR: File Open" <<fileName<< '\n';  
    vector<string> Avectorstring;
    string astring;
    int row = 0;
    int col = 0;
  
    while(ip.good())
    {
        if(getline(ip,astring,'\n'))
        {
            row++;
            Avectorstring.push_back(astring);
            col = count(astring.begin(), astring.end(), ',')+1;
            astring.clear();
        }
    }
    // std::cout<<"Row: "<<row<<" "<<"Col: "<<col<<std::endl;
    vector<vector<double>> Amatrixdouble (row,vector<double> (col));
    // vector<vector<string>> Amatrixstring (row,vector<string> (col));
    for(int i=0;i<row;i++) //jumlah baris
    {
        std::replace(Avectorstring[i].begin(), Avectorstring[i].end(), ',', ' '); 
        std::istringstream iss(Avectorstring[i]);
        std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
                                 std::istream_iterator<std::string>());
        for(int j=0;j<col;j++) //jumlah kolom
        {
            // Amatrixstring[i].push_back(results[j]);
            Amatrixdouble[i][j]=stod(results[j]);
        }
    }
    Avectorstring.clear();
    // for(int i=0; i<Amatrixstring.size(); i++)
    // {
    //     for(int j=0; j<Amatrixstring[i].size(); j++)
    //     {
    //         std::cout<<Amatrixstring[i][j]<<" ";
    //     }
    //     std::cout<<"\n";
    // }

    // for(int i=0; i<Amatrixdouble.size(); i++)
    // {
    //     for(int j=0; j<Amatrixdouble[i].size(); j++)
    //     {
    //         std::cout<<Amatrixdouble[i][j]<<" ";
    //     }
    //     std::cout<<"\n";
    // }
    ip.close();
    return Amatrixdouble;
}

std::vector<std::vector<double> > csvManager::transpose(const std::vector<std::vector<double> > data) {
    // this assumes that all inner vectors have the same size and
    // allocates space for the complete result in advance
    std::vector<std::vector<double> > result(data[0].size(),
                                          std::vector<double>(data.size()));
    for (std::vector<double>::size_type i = 0; i < data[0].size(); i++) 
        for (std::vector<double>::size_type j = 0; j < data.size(); j++) {
            result[i][j] = data[j][i];
        }
    return result;
}

}