#ifndef GVP_EXCEPTIONS
#define GVP_EXCEPTIONS

#include <exception>
#include <string>

namespace GVP
{
    class SearchError : public std::exception
    {
    private:
        std::string message_;

    public:
        SearchError(const std::string& message) : message_(message) {}
        
        const char* what() const throw()
        {
            return message_.c_str();
        }
    };
}

#endif //GVP_EXCEPTIONS
