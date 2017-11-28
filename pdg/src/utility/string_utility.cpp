#include <utility/tf_utility.h>

bool splitStringInTwo(const std::string str, std::string& str1, std::string& str2, const std::string sep, const std::string pref)
{
    std::string s = str;
    // Check and remove tf prefix if valid
    if ( s != pref &&
         s.size() > pref.size() &&
         s.substr(0, pref.size()) == pref )
    {
        // s start with tf_prefix
        s = s.substr(pref.size(), s.size());
    }
    else
    {
        return false;
    }

    // Split the remaining string
    str1 = s.substr(0, s.find(sep));
    str2 = s.substr(s.find(sep) + sep.size(), s.size());

    // Check the validity of the strings
    if(str1.empty()  ||  str2.empty())
    {
        std::cerr<<"str1 |& str2 frames cannot be set from "<<s<<" string"<<std::endl;
        return false;
    }

    return true;
}
