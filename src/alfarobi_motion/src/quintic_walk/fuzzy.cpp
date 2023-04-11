#include "quintic_walk/fuzzy.hpp"

void Fuzzy::setMembership(int amountData, std::vector<double> &Bottom1, std::vector<double> &Upper1, std::vector<double> &Upper2, std::vector<double> &Bottom2)
{
    dataAmount = amountData;

    for(int i=0; i<dataAmount; i++)
    {
        bottom1[i] = Bottom1[i];
        upper1[i] = Upper1[i];
        upper2[i] = Upper2[i];
        bottom2[i] = Bottom2[i];
    }
}

void Fuzzy::setRule(int row, int col, const Eigen::MatrixXi &data)
{
    for(int i=0; i<10; i++)
        for(int j=0; j<10; j++)
            rule[i][j]=0;

    for(int i=0; i<row; i++)
        for(int j=0; j<col; j++)
            rule[i][j]=data(i,j);
}

void Fuzzy::printRule()
{
    // std::cout<<"\nRULE"<<std::endl;

    for(int i=0; i<10; i++)
    {
        for(int j=0; j<10; j++)
            std::cout<<" "<<rule[i][j];
        std::cout<<std::endl;
    }
}

double Fuzzy::calcX(double y, double y1, double y2, double x1, double x2)
{
    // Persamaan garis

    //   x - x1          y - y1
    // ---------   =   ---------
    //  x2 - x1         y2 - y1    
    //                (Y_section)

    // jadi:
    //       y - y1 
    // x = --------- ( x2 - x1 ) + x1
    //      y2 - y1 

    double Y_section = (y - y1) / (y2 - y1);

    return ((x2 - x1) * Y_section) + x1;
}

double Fuzzy::calcY(double x, double x1, double x2, double y1, double y2)
{
    // Persamaan garis

    //   x - x1          y - y1
    // ---------   =   ---------
    //  x2 - x1         y2 - y1    
    // (X_section)

    // jadi:
    //       x - x1 
    // y = --------- ( y2 - y1 ) + y1
    //      x2 - x1 

    double X_section = (x - x1) / (x2 - x1);

    return ((y2 - y1) * X_section) + y1;
}

double Fuzzy::calcHypotenuse(double x1, double y1, double x2, double y2)
{
    // c = sqrt( alas^2 + tinggi^2)
    // c = sqrt((x2 - x1)^2 + (y2 - y1)^2)

    double x = x2-x1;
    double y = y2-y1;

    return sqrt(pow(x,2)+pow(y,2));
}

double Fuzzy::calcGradien(double x1, double y1, double x2, double y2)
{
    //      y2 - y1
    // m = ---------
    //      x2 - x1

    double x = x2-x1;
    double y = y2-y1;

    return y/x;
}

Eigen::ArrayXd Fuzzy::calcUnion(int section, double value)
{
    Eigen::ArrayXd area(3);
    area.setZero();
    
    double base = bottom2[section]-bottom1[section];
    // std::cout<<"\nBase : "<<base<<std::endl;
    double roof = (1-value) * base;
    // std::cout<<"Roof : "<<roof<<std::endl;
    double x2_c, x2_d, c, d, nominator, denominator;

    // AREA(0) -> Luas
    // AREA(1) -> CoM sumbu X
    // AREA(2) -> CoM sumbu Y

    if(value == 1 && upper1[section] ==  upper2[section])
    {
        // SEGITIGA
        
        //         alas * tinggi
        // Luas = ----------------
        //              2
        area(0) = base/2;
        // std::cout<<"Area Segitiga : "<<area(0)<<std::endl;
        
        // CoM X = pertemuan CoM Y dengan garis dari titik tengah base ke titik atas
        // area(1) = calcX(COM_Y_TRIANGLE, 0, 1, bottom1[section]+(base/2), upper1[section]);

        // CoM X = gabungan koordinat x / 3
        area(1) = (bottom1[section]+upper1[section]+bottom2[section])/3;
        // std::cout<<"X Segitiga : "<<area(1)<<std::endl;
        
        // CoM Y = selalu bernilai 1/3 untuk segala jenis segitiga
        area(2) = COM_Y_TRIANGLE;
    }
    else if(value != 0)
    {
        // TRAPESIUM

        //         (alas + atap) * tinggi
        // Luas = ------------------------
        //                   2
        // atap = (1 - value) * alas
        area(0) = (base + roof) * value / 2;
        // std::cout<<"Area TRAPESIUM : "<<area(0)<<std::endl;

        //          alas     (2atap + alas) * (c^2 - d^2) 
        // COM X = ------ + -------------------------------
        //            2          6 * (alas^2 - atap^2)

        // jadi:
        //          alas      nominator 
        // COM X = ------ + -------------
        //            2      denominator

        x2_c = calcX(value, 0, 1, bottom1[section], upper1[section]);
        // std::cout<<"C TRAPESIUM : "<<x2_c<<std::endl;
        x2_d = calcX(value, 1, 0, upper1[section], bottom2[section]);
        // std::cout<<"D TRAPESIUM : "<<x2_d<<std::endl;
        c = calcHypotenuse(bottom1[section], 0, x2_c, value);
        d = calcHypotenuse(x2_d, value, bottom2[section], 0);

        nominator = (2*roof + base) * (pow(c,2) - pow(d,2));
        denominator = 6 * (pow(base, 2) - pow(roof, 2));
        area(1) = bottom1[section] + (base / 2) + (nominator / denominator);
        // std::cout<<"X TRAPESIUM : "<<area(1)<<std::endl;
        
        //          alas + (2 * atap)
        // COM Y = ------------------- * tinggi
        //           3 (alas + atap)

        //           nominator
        // COM Y = ------------- * tinggi
        //          denominator

        nominator = base + (2 * roof);
        denominator = 3 * (base + roof);

        area(2) = nominator / denominator * value;
    }

    return area;
}

Eigen::ArrayXd Fuzzy::calcIntersection(const Eigen::ArrayXi &section, const Eigen::ArrayXd &valueSection)
{
    Eigen::ArrayXd area(3);
    area.setZero();

    double base = bottom2[section(0)] - bottom1[section(1)];
    double value, roof, x2_c, x2_d, c, d, nominator, denominator;
    double m[2], b[2];

    m[1] = calcGradien(upper1[section(0)], 1, bottom2[section(0)], 0);
    m[2] = calcGradien(bottom1[section(1)], 0, upper1[section(1)], 1);

    // Persamaan Garis 1
    // y = (m1 * x) + b1
    // Persamaan Garis 2
    // y = (m2 * x) + b2

    // b = y - (m * x)
    b[1] = 1 - (m[1] * upper1[section(0)]);
    b[2] = 0 - (m[2] * bottom1[section(1)]);

    // (m1 * x) + b1 = (m2 * x) + b2

    // x * (m1 - m2) = b2 - b1

    //       b2 - b1
    // x = -----------
    //       m1 - m2
    double x = (b[2] - b[1])/(m[1] - m[2]);
    double y = calcY(x, upper1[section(0)], bottom2[section(0)], 1, 0);

    // std::cout<<"PERPOTONGAN X : "<<x<<" , Y :"<<y<<std::endl;

    // AREA(0) -> Luas
    // AREA(1) -> CoM sumbu X
    // AREA(2) -> CoM sumbu Y

    if(y <= valueSection(0) && y <= valueSection(1))
    {
        // SEGITIGA
        
        //         alas * tinggi
        // Luas = ----------------
        //              2
        area(0) = y*base/2;
        
        // CoM X = pertemuan CoM Y dengan garis dari titik tengah base ke titik atas
        // area(1) = calcX(COM_Y_TRIANGLE, 0, 1, bottom1[section]+(base/2), upper1[section]);

        // CoM X = gabungan koordinat x / 3
        area(1) = (bottom1[section(0)]+x+bottom1[section(1)])/3;
        
        // CoM Y = selalu bernilai 1/3 untuk segala jenis segitiga
        area(2) = COM_Y_TRIANGLE*y;
    }
    else
    {
        // TRAPESIUM

        value = (valueSection(0)<valueSection(1)) ? valueSection(0) : valueSection(1);
        roof = calcX(value, 1, 0, upper1[section(0)], bottom2[section(0)]) - calcX(value, 0, 1, bottom1[section(1)], upper1[section(1)]);

        // std::cout<<"VALUE : "<<value<<std::endl;
        // std::cout<<"ROOF : "<<roof<<std::endl;

        //         (alas + atap) * tinggi
        // Luas = ------------------------
        //                   2
        // atap = (1 - value) * alas
        area(0) = (base + roof) * value / 2;

        //          alas     (2atap + alas) * (c^2 - d^2) 
        // COM X = ------ + -------------------------------
        //            2          6 * (alas^2 - atap^2)

        // jadi:
        //          alas      nominator 
        // COM X = ------ + -------------
        //            2      denominator

        x2_c = calcX(value, 0, y, bottom1[section(1)], x);
        x2_d = calcX(value, y, 0, x, bottom2[section(0)]);
        c = calcHypotenuse(bottom1[section(1)], 0, x2_c, value);
        d = calcHypotenuse(x2_d, value, bottom2[section(0)], 0);

        nominator = (2*roof + base) * (pow(c,2) - pow(d,2));
        denominator = 6 * (pow(base, 2) - pow(roof, 2));
        area(1) = bottom1[section(1)] + (base / 2) + (nominator / denominator);

        //          alas + (2 * atap)
        // COM Y = ------------------- * tinggi
        //           3 (alas + atap)

        //           nominator
        // COM Y = ------------- * tinggi
        //          denominator

        nominator = base + (2 * roof);
        denominator = 3 * (base + roof);

        area(2) = nominator / denominator * value;
    }

    return area;
}

Eigen::ArrayXd Fuzzy::predicate()
{
    Eigen::ArrayXi level(3);
    Eigen::ArrayXd value(dataAmount);
    level.setZero();
    value.setZero();

    // level(0) -> bottomLevel 
    // level(1) -> middleLevel
    // level(2) -> upperLevel

    // bottomLevel -> membership yang ada di dalam rentang tanjakan trapesium
    // middleLevel -> membership yang ada di dalam rentang atap trapesium (2 data atas)
    // upperLevel  -> membership yang ada di dalam rentang turunan trapesium

    for(int i = 0; i < dataAmount; i++)
    {
        //upperLevel
        if(input >= bottom1[i] && input < upper1[i])
            level(0) = i+1;
        //middleLevel
        if(input >= upper1[i] && input <= upper2[i])
            level(1) = i+1;
        //upperLevel
        if(input > upper2[i] && input <= bottom2[i])
            level(2) = i+1;
    }

    // std::cout<<"\nInput : "<<input<<"\t Bottom Level : "<<level(0)<<"\t Upper  Level: "<<level(1)<<std::endl;

    for(int i = 1; i <= dataAmount; i++)
    {
        if(level(0)==i)
            value(level(0)-1) = calcY(input, bottom1[i-1], upper1[i-1], 0, 1);

        if(level(2)==i)
            value(level(2)-1) = calcY(input, upper2[i-1], bottom2[i-1], 1, 0);
        
        if(level(1)==i)
            value(level(1)-1) = 1;
    }

    // std::cout<<"PREDICATE"<<std::endl;
    // std::cout<<value<<std::endl;

    return value;
}

Eigen::ArrayXd Fuzzy::membership(const Eigen::ArrayXd &predicate1, const Eigen::ArrayXd &predicate2, int &dataAmount1, int &dataAmount2)
{
    Eigen::ArrayXd inferenceOut(dataAmount);
    inferenceOut.setZero();
    double k;

    for (int i = 0; i < dataAmount1; i++)
    {
        for (int j = 0; j < dataAmount2; j++)
        {
            k = rule[i][j] - 1;
            inferenceOut(k) = std::fmax(inferenceOut(k), std::fmin(predicate1(i), predicate2(j)));
        }
    }

    return inferenceOut;
}

double Fuzzy::outputCentroid(const Eigen::ArrayXd &membership)
{
    // Eigen::ArrayXd cekArea(dataAmount);
    // Eigen::ArrayXd cekCOMX(dataAmount);
    // Eigen::ArrayXd cekCOMY(dataAmount);
    Eigen::ArrayXd area(3);
    Eigen::ArrayXd area_(3);
    Eigen::ArrayXi section(2);
    Eigen::ArrayXd value(2);
    // cekArea.setZero();
    // cekCOMX.setZero();
    // cekCOMY.setZero();
    area.setZero();
    area_.setZero();

    // area(0) = area_(0) -> Luas
    // area(1) = area_(0) -> CoM sumbu X
    // area(2) = area_(0) -> CoM sumbu Y

    // std::cout<<"\nMEMBERSHIP"<<std::endl;
    // std::cout<<membership<<std::endl;

    for(int i = 0; i < dataAmount; i++)
    {
        // cekArea(i) = area(0);
        // cekCOMX(i) = area(1);
        // cekCOMY(i) = area(2);
        area = calcUnion(i, membership(i));
        area_(0) += area(0);
        area_(1) += (area(0) * area(1));
        area_(2) += (area(0) * area(2));
    }

    // std::cout<<"\nUNION AREA"<<std::endl;
    // std::cout<<cekArea<<std::endl;
    // std::cout<<"\nCOM X"<<std::endl;
    // std::cout<<cekCOMX<<std::endl;
    // std::cout<<"\nCOM Y"<<std::endl;
    // std::cout<<cekCOMY<<std::endl;

    // cekArea.setZero();
    // cekCOMX.setZero();
    // cekCOMY.setZero();
    area.setZero();

    for(int i = 0; i < (dataAmount - 1); i++)
    {
        section << i, (i+1);
        value << membership(i), membership(i+1);
        if(membership(i) != 0 && membership(i+1) != 0)
            area = calcIntersection(section, value);
        // cekArea(i) = area(0);
        // cekCOMX(i) = area(1);
        // cekCOMY(i) = area(2);
        area_(0) -= area(0);
        area_(1) -= (area(0) * area(1));
        area_(2) -= (area(0) * area(2));
        area.setZero();
    }

    // std::cout<<"\nINTERSECTION AREA"<<std::endl;
    // std::cout<<cekArea<<std::endl;
    // std::cout<<"\nCOM X"<<std::endl;
    // std::cout<<cekCOMX<<std::endl;
    // std::cout<<"\nCOM Y"<<std::endl;
    // std::cout<<cekCOMY<<std::endl;

    // area_(1) /= area_(0);
    // area_(2) /= area_(0);

    // std::cout<<"\nCOM"<<std::endl;
    // std::cout<<area_<<std::endl;

    //output Fuzzy hanay butuh CoM sumbu X

    // if(area_(0) == 0)
    //     std::cout<<"\nHASIL FUZZY : 0"<<std::endl;
    // else
    //     std::cout<<"\nHASIL FUZZY : "<<area_(1) / area_(0)<<std::endl;
  
    if(area_(0) == 0 || area_(1) == 0)
        return 0;
    else
        return area_(1) / area_(0);
}