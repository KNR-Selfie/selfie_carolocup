#include <path_planner/polyfit.hpp>


poly::poly()
{

}
/*
    Finds the coefficients of a polynomial p(x) of degree n that fits the data,
    p(x(i)) to y(i), in a least squares sense. doublehe result p is a row vector of
    length n+1 containing the polynomial coefficients in incremental powers.

    param:
        oX				x axis values
        oY				y axis values
        nDegree			polynomial degree including the constant

    return:
        coefficients of a polynomial starting at the constant coefficient and
        ending with the coefficient of power to nDegree. C++0x-compatible
        compilers make returning locally created vectors very efficient.

*/

void poly::polyfit(int nDegree )
{
    using namespace boost::numeric::ublas;

    if ( x_raw_pts.size() != y_raw_pts.size() )
        throw std::invalid_argument( "X and Y vector sizes do not match" );

    // more intuative this way
    nDegree++;

    size_t nCount =  x_raw_pts.size();
    matrix<float> oXMatrix( nCount, nDegree );
    matrix<float> oYMatrix( nCount, 1 );

    // copy y matrix
    for ( size_t i = 0; i < nCount; i++ )
    {
        oYMatrix(i, 0) = y_raw_pts[i];
    }

    // create the X matrix
    for ( size_t nRow = 0; nRow < nCount; nRow++ )
    {
        float nVal = 1.0f;
        for ( int nCol = 0; nCol < nDegree; nCol++ )
        {
            oXMatrix(nRow, nCol) = nVal;
            nVal *= x_raw_pts[nRow];
        }
    }

    // transpose X matrix
    matrix<float> oXtMatrix( trans(oXMatrix) );
    // multiply transposed X matrix with X matrix
    matrix<float> oXtXMatrix( prec_prod(oXtMatrix, oXMatrix) );
    // multiply transposed X matrix with Y matrix
    matrix<float> oXtYMatrix( prec_prod(oXtMatrix, oYMatrix) );

    // lu decomposition
    permutation_matrix<int> pert(oXtXMatrix.size1());
    const std::size_t singular = lu_factorize(oXtXMatrix, pert);

    // must be singular
    BOOST_ASSERT( singular == 0 );

    // backsubstitution
    lu_substitute(oXtXMatrix, pert, oXtYMatrix);

    // copy the result to coeff

    std::vector<float>vec( oXtYMatrix.data().begin(), oXtYMatrix.data().end() );
    coeff=vec;
}

/*
    Calculates the value of a polynomial of degree n evaluated at x. doublehe input
    argument pCoeff is a vector of length n+1 whose elements are the coefficients
    in incremental powers of the polynomial to be evaluated.

    param:
        oCoeff			polynomial coefficients generated by polyfit() function
        oX				x axis values

    return:
        Fitted Y values. C++0x-compatible compilers make returning locally
        created vectors very efficient.
*/

void poly::polyval()
{
    size_t nCount =  x_raw_pts.size();
    size_t nDegree = coeff.size();
    std::vector<float>output( nCount );

    for ( size_t i = 0; i < nCount; i++ )
    {
        double nY = 0;
        double nXdouble = 1;
        double nX = x_raw_pts[i];
        for ( size_t j = 0; j < nDegree; j++ )
        {
            // multiply current x by a coefficient
            nY += coeff[j] * nXdouble;
            // power up the X
            nXdouble *= nX;
        }
        output[i] = nY;
    }
    y_output_pts = output;
}

void poly::get_row_pts(const std::vector<geometry_msgs::Point> point_vec)
{
    x_raw_pts.clear();
    y_raw_pts.clear();

    for(int i = 0;i<point_vec.size();i++)
    {
        x_raw_pts.push_back(point_vec[i].x);
        y_raw_pts.push_back(point_vec[i].y);
    }

}

void poly::find_middle(poly left, poly right)
{
    //reduce point number
    x_raw_pts.reserve( left.x_raw_pts.size() + right.x_raw_pts.size()); // preallocate memory
    x_raw_pts.insert( x_raw_pts.end(), left.x_raw_pts.begin(), left.x_raw_pts.end() );
    x_raw_pts.insert( x_raw_pts.end(), right.x_raw_pts.begin(), right.x_raw_pts.end() );

    y_raw_pts.reserve( left.y_raw_pts.size() + right.y_raw_pts.size()); // preallocate memory
    y_raw_pts.insert( y_raw_pts.end(), left.y_raw_pts.begin(), left.y_raw_pts.end() );
    y_raw_pts.insert( y_raw_pts.end(), right.y_raw_pts.begin(), right.y_raw_pts.end() );

    this->polyfit(7);
    this->polyval();
}

tangent::tangent()
{

}
void tangent::calc(poly polynom,float x)
{
    float value = 0;
    int degree = polynom.coeff.size();
    for(int i = 0;i<degree;i++)
    {
        value += i*polynom.coeff[i]*pow(x,i-1);
    }

}
