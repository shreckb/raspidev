//==============================================================================
/// @file TimeStamp.cc
//------------------------------------------------------------------------------
// author Cyril OTHENIN-GIRARD <cog@free.fr>
// date   2009-09 (Creation)
//==============================================================================

#include "TimeStamp.h"

#include <sstream>
#include <iomanip>
#include <limits>
#include <array>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cassert>

//#include <boost/algorithm/string.hpp>

namespace chrono = std::chrono;

static constexpr auto MAX_FRACTIONAL_DIGITS = 9;

//------------------------------------------------------------------------------
TimeStamp const& TimeStamp::INVALID() {
    static TimeStamp const s__constant{std::numeric_limits<raw_type>::min(), TimeStamp::RAW};
    return s__constant;
}
TimeStamp const& TimeStamp::ZERO() {
    static TimeStamp const s__constant{0, TimeStamp::RAW};
    return s__constant;
}
TimeStamp const& TimeStamp::OO() {
    static TimeStamp const s__constant{std::numeric_limits<raw_type>::max(), TimeStamp::RAW};
    return s__constant;
}
TimeStamp const& TimeStamp::ACCURACY() {
    static TimeStamp const s__constant{1, TimeStamp::RAW};
    return s__constant;
}

#ifdef _MSC_VER
//------------------------------------------------------------------------------
// Sous Windows, localtime() et gmtime() sont threadsafe (car leur buffer
// interne est alloue dans le "thread-local storage").
// Donc on peut les utiliser pour implementer les fonctions re-entrantes
// equivalentes d'UNIX
#include <ctime>    // == <time.h>
static struct tm *
localtime_r(time_t const* timer, struct tm* result)
{
    if (0 == result) return nullptr;
    *result = *localtime(timer);
    return result;
}
static struct tm *
gmtime_r(time_t const* timer, struct tm* result)
{
    if (0 == result) return nullptr;
    *result = *gmtime(timer);
    return result;
}
#endif

//------------------------------------------------------------------------------
time_t
TimeStamp::AsSystemTime(
    double* p_fractional_seconds,
    int     fractional_digits
    ) const
{
    static constexpr
        std::array<int64_t, MAX_FRACTIONAL_DIGITS + 1> POW10 = {
                           1LL,     // 0 digit
                          10LL,     // 1 digit
                         100LL,     // 2 digits
                        1000LL,     // 3 ...
                   10LL*1000LL,     // 4 ...
                  100LL*1000LL,     // 5 ...
                 1000LL*1000LL,     // 6 ...
            10LL*1000LL*1000LL,     // 7 ...
           100LL*1000LL*1000LL,     // 8 ...
          1000LL*1000LL*1000LL   }; // 9 ...
    static_assert(POW10.back() == FREQUENCY, "");

    auto result = std::div(counter_, FREQUENCY);
    if (p_fractional_seconds) {
        if (fractional_digits < 0) fractional_digits = MAX_FRACTIONAL_DIGITS;
        else                       fractional_digits = std::min(fractional_digits, MAX_FRACTIONAL_DIGITS);
        *p_fractional_seconds = result.rem / POW10[MAX_FRACTIONAL_DIGITS - fractional_digits];
        *p_fractional_seconds /= double(POW10[fractional_digits]);
    }
    return static_cast<time_t>(result.quot);
}

#ifndef TIMESTAMP__NO_DEPRECATED
//------------------------------------------------------------------------------
void
TimeStamp::AsLocalDateTime(
    std::string* p_date,
    std::string* p_time,
    int          fractional_digits
    ) const
{
    fractional_digits = std::min(fractional_digits, MAX_FRACTIONAL_DIGITS);

    double       fractional_seconds;
    std::time_t  sys_time = this->AsSystemTime(&fractional_seconds, fractional_digits);
    std::tm      tm_time;
    localtime_r(&sys_time, &tm_time);

    char sz_buffer[32];

    if (nullptr != p_date) {
        std::snprintf(sz_buffer, 31,
                        "%04d/%02d/%02d",           // YYYY/MM/DD
                        tm_time.tm_year + 1900,
                        tm_time.tm_mon  + 1,
                        tm_time.tm_mday
                        );
        sz_buffer[31] = '\0';
        *p_date = sz_buffer;
    }

    if (nullptr != p_time) {
        int seconds_width = 2 + fractional_digits;
        if (fractional_digits > 0) seconds_width ++;  // for decimal point !

        std::snprintf(sz_buffer, 31,
                        "%02d:%02d:%0*.*f",        // HH:MM:SS.sss
                        tm_time.tm_hour,
                        tm_time.tm_min,
                        seconds_width,
                        fractional_digits,
                        tm_time.tm_sec + fractional_seconds
                        );
        sz_buffer[31] = '\0';
        *p_time = sz_buffer;
    }
}
#endif

//------------------------------------------------------------------------------
std::string
TimeStamp::AsIsoDateTime(
    EDateTimeType format,
    int           fractional_digits,
    bool          with_TZD
    ) const
{
    if (not IsValid()) {
        return "invalid";
    }

    fractional_digits = std::min(fractional_digits, MAX_FRACTIONAL_DIGITS);

    double      fractional_seconds;
    std::time_t sys_time = this->AsSystemTime(&fractional_seconds, fractional_digits);
    std::tm     tm_time;
    if (DATETIME_UTC == format) {
        gmtime_r(&sys_time, &tm_time);
    } else {
        localtime_r(&sys_time, &tm_time);
    }

    int seconds_width = 2 + fractional_digits;
    if (fractional_digits > 0) ++seconds_width;    // for decimal point

    std::string time_zone_designator;  // default: empty string
    if (with_TZD) {
        if (DATETIME_UTC == format) {
            time_zone_designator = "Z";
        } else {
            char buf[16];
            auto size = std::strftime(buf, sizeof(buf), "%z", &tm_time);
            if (0 == size) { // error !
//                THROW_RUNTIME_ERROR(
                throw std::runtime_error(
                    "Unable to get TSD (Time Zone Designator)"
                    " when converting timestamp to local date/time."
                );
            } else {
                time_zone_designator = buf;
            }
        }
    }

    char sz_buffer[128];
    std::snprintf(sz_buffer, 127,
                "%04d-%02d-%02dT%02d:%02d:%0*.*f%s",
                tm_time.tm_year + 1900,
                tm_time.tm_mon + 1,
                tm_time.tm_mday,
                tm_time.tm_hour,
                tm_time.tm_min,
                seconds_width,
                fractional_digits,
                tm_time.tm_sec + fractional_seconds,
                time_zone_designator.c_str()
                );
    sz_buffer[127] = '\0';
    return std::string{sz_buffer};
}

//------------------------------------------------------------------------------
/// @todo TODO TimeStamp::Set(ISO_DATE_TIME) - Add parsing of TZD (using POSIX strptime() ?)
/// @todo (?) Use the following snipet to reimplement TimeStamp ctor from an ISO date/time string
/*
    #include <boost/date_time/posix_time/posix_time.hpp>

    std::string iso_date_time;
    boost::posix_time::ptime t(boost::posix_time::from_iso_string(iso_date_time));
    struct tm my_tm = boost::posix_time::to_tm(t);
    TimeStamp tsp(TimeStamp::SYSTEM_TIME, mktime(&my_tm), 0);
*/
void
TimeStamp::Set(EIsoDateTime, std::string const& xml)
{
    Invalidate();

    std::size_t debut = 0;
    std::size_t fin   = 0;
    char* p_end;

    // Récupération de l'année
    fin = xml.find_first_of("-");
    if (fin == xml.npos)  return;
    int year = strtol(xml.substr(debut, fin - debut).c_str(), &p_end, 10);
    if (strcmp(p_end, "") != 0)  return;

    // Récupération du mois
    debut = fin + 1;
    fin = xml.find_first_of("-" , debut);
    if (fin == xml.npos)  return;
    int month = strtol(xml.substr(debut, fin - debut).c_str(), &p_end, 10);
    if (strcmp(p_end, "") != 0)  return;

    // Récupération du jour
    debut = fin + 1;
    fin = xml.find_first_of("T", debut);
    if (fin == xml.npos)  return;
    int day = strtol(xml.substr(debut, fin - debut).c_str(), &p_end, 10);
    if (strcmp(p_end, "") != 0)  return;

    // Récupération de l'heure
    debut = fin + 1;
    fin = xml.find_first_of(":", debut);
    if (fin == xml.npos)  return;
    int hour = strtol(xml.substr(debut, fin - debut).c_str(), &p_end, 10);
    if (strcmp(p_end, "") != 0)  return;

    // Récupération des minutes
    debut = fin + 1;
    fin = xml.find_first_of(":", debut);
    if (fin == xml.npos)  return;
    int minute = strtol(xml.substr(debut, fin - debut).c_str(), &p_end, 10);
    if (strcmp(p_end, "") != 0)  return;

    // Récupération des secondes
    debut = fin + 1;
    fin = xml.find_first_of("+Z", debut);
//    if (fin == xml.npos)  return;
    if (fin == xml.npos) fin = xml.size();
    double seconds = strtod(xml.substr(debut, fin - debut).c_str(), &p_end);
    if (strcmp(p_end, "") != 0)  return;

    // transformation en timestamp
    std::tm  tm_time;
    tm_time.tm_year  = year-1900;
    tm_time.tm_mon   = month-1;
    tm_time.tm_mday  = day;
    tm_time.tm_hour  = hour;
    tm_time.tm_min   = minute;
    tm_time.tm_sec   = static_cast<int>(seconds);
    tm_time.tm_isdst = -1;

    double fractional_part = seconds - static_cast<int>(seconds);
    this->Set(SYSTEM_TIME, mktime(&tm_time), fractional_part);
}

//------------------------------------------------------------------------------
std::ostream &
operator << (std::ostream & r_out, TimeStamp const& timestamp) {
    if (not timestamp.IsValid())       return r_out << "<invalid>";
    if (timestamp ==  TimeStamp::OO()) return r_out << "+OO";
    if (timestamp == -TimeStamp::OO()) return r_out << "-OO";
    return r_out << std::fixed << std::showpoint
                 << std::setprecision(3) << timestamp.Seconds();
}

//------------------------------------------------------------------------------
std::string
ToString(TimeStamp const& timestamp) {
    std::ostringstream oss;
    oss << timestamp;
    return oss.str();
}

//------------------------------------------------------------------------------
std::istream&
operator >> (std::istream & r_in, TimeStamp& r_timestamp) {
    double d;
    r_in >> d;
    if (r_in.good() or r_in.eof()) r_timestamp.Set(d, TimeStamp::SECOND);
    else                           r_timestamp.Invalidate();
    return r_in;
}
