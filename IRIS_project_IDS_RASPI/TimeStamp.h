#pragma once

//==============================================================================
/// @file TimeStamp.h
// author Cyril OTHENIN-GIRARD <cog@free.fr>
// date   2009-09 (Creation)
//==============================================================================

#include <cstdint>
#include <cmath>
#include <string>
#include <type_traits>
#include <stdexcept>
#include <iosfwd>
#include <functional>     // for std::hash

#include <chrono>

//#define TIMESTAMP__NO_DEPRECATED
//#define TIMESTAMP__NO_STD_CHRONO_ITEROPERABILITY

#if __cplusplus <= 201402L
// In C++11 and C++14, adding C++17 overload of std::abs() for std::chrono::duration<...>
namespace std {
    template <class Rep, class Period, class = enable_if_t<
        chrono::duration<Rep, Period>::min() < chrono::duration<Rep, Period>::zero()>>
    constexpr chrono::duration<Rep, Period> abs(chrono::duration<Rep, Period> d)
    {
        return d >= d.zero() ? d : -d;
    }
}
#endif

//==============================================================================
class TimeStamp
{
public:
    using duration = std::chrono::nanoseconds;
    using raw_type = duration::rep;

    static_assert(std::is_same<raw_type, std::int64_t>{}, "");

private:
    raw_type counter_;

    /// @name Implementation constants
    //@{
    static constexpr raw_type FREQUENCY = duration::period::den;
    static_assert(FREQUENCY == 1'000'000'000LL, "");
    //@}
public:

    /// @name Constants
    //@{
    static TimeStamp const& INVALID();
    static TimeStamp const& ZERO();
    static TimeStamp const& OO();
    static TimeStamp const& ACCURACY();
    //@}

    /// Default ctor : creates an invalid timestamp
    TimeStamp() noexcept { Invalidate(); }

    /// @name Validity
    //@{
    void Invalidate() noexcept { *this = INVALID(); }
    bool IsValid() const noexcept;
    void AssertValid() const {
            if (not IsValid()) {
                throw std::runtime_error("Invalid timestamp");
            }
        }
    //@}

    /// Construit un timestamp initialise sur l'instant present
    enum ENow { NOW };
    explicit TimeStamp(ENow) { Set(NOW); }
    void Set(ENow) {
        // counter = duration since epoch (midnight (00:00:00), January 1, 1970 (UTC))
        auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
        counter_ = std::chrono::duration_cast<duration>(since_epoch).count();
    }
    static TimeStamp Now() { return TimeStamp(NOW); }

    /// Construit un timestamp a partir d'un delai en jours
    enum EDay { DAY = 0 };
    TimeStamp(double days, EDay)    { Set(days*24.0*3600.0, SECOND); }
    void Set(double days, EDay)     { Set(days*24.0*3600.0, SECOND); }

    /// Construit un timestamp a partir d'un delai en heures
    enum EHour { HOUR = 0 };
    TimeStamp(double hours, EHour)  { Set(hours*3600.0, SECOND); }
    void Set(double hours, EHour)   { Set(hours*3600.0, SECOND); }

    /// Construit un timestamp a partir d'un delai en minutes
    enum EMinute { MINUTE = 0 };
    TimeStamp(double minutes, EMinute)  { Set(minutes*60.0, SECOND); }
    void Set(double minutes, EMinute)   { Set(minutes*60.0, SECOND); }

    /// Construit un timestamp a partir d'un delai en secondes
    enum ESecond { SECOND = 0 };
    TimeStamp(double seconds, ESecond)  { Set(seconds, SECOND); }
    void Set(double seconds, ESecond) {
        counter_ = static_cast<raw_type>(std::round(seconds*FREQUENCY));
    }

    /// Construit un timestamp a partir d'un delai en millisecondes
    enum EMillisecond { MILLISECOND = 3 };
    TimeStamp(double milliseconds, EMillisecond) { Set(milliseconds/1000.0, SECOND); }
    void Set(double milliseconds, EMillisecond)  { Set(milliseconds/1000.0, SECOND); }

    /// Construit un timestamp a partir d'un delai en microsecondes
    enum EMicrosecond { MICROSECOND = 6 };
    TimeStamp(double microseconds, EMicrosecond) { Set(microseconds/1000.0/1000.0, SECOND); }
    void Set(double microseconds, EMicrosecond)  { Set(microseconds/1000.0/1000.0, SECOND); }

    /// Construit un timestamp a partir d'un delai en unite interne
    enum ERaw { RAW };
    constexpr TimeStamp(raw_type raw_value, ERaw) : counter_{raw_value} {}
    void Set(raw_type raw_value, ERaw) { counter_ = raw_value; }

    /// Construit un timestamp a partir d'un temps systeme (time_t + reste sous la seconde)
    enum ESystemTime { SYSTEM_TIME };
    TimeStamp(ESystemTime, time_t seconds, double fractional_seconds = 0.0)
        { Set(SYSTEM_TIME, seconds, fractional_seconds); }
    void Set(ESystemTime, time_t seconds, double fractional_seconds = 0.0) {
        Set(seconds*FREQUENCY, RAW);
        *this += TimeStamp(fractional_seconds, SECOND);
    }

    /// Construct a timestamp from a date/time string in ISO 8601 extended format (example: "2016-08-26T21:37:54.123")
    enum EIsoDateTime { ISO_DATE_TIME };
    TimeStamp(EIsoDateTime, std::string const& iso_date_time)
        { Set(ISO_DATE_TIME, iso_date_time); }
    void Set(EIsoDateTime, std::string const& iso_date_time);

    /// Renvoie la valeur du timestamp en jours
    double Days() const noexcept    { return this->Seconds()/3600.0/24.0; }

    /// Renvoie la valeur du timestamp en heures
    double Hours() const noexcept   { return this->Seconds()/3600.0; }

    /// Renvoie la valeur du timestamp en minutes
    double Minutes() const noexcept { return this->Seconds()/60.0; }

    /// Renvoie la valeur du timestamp en secondes
    double Seconds() const noexcept {
        AssertValid();
        return double(counter_)/FREQUENCY;
//        return AsDuration<std::chrono::duration<double>>().count();
    }

    /// Renvoie la valeur du timestamp en millisecondes
    double MilliSeconds() const noexcept {
        return 1000.0*this->Seconds();
    }

    /// Renvoie la valeur du timestamp en microsecondes
    double MicroSeconds() const noexcept {
        return 1'000'000.0*this->Seconds();
    }

    /// Conversion en temps systeme.
    /// @remarks On both UNIX and WINDOWS, 'system time' is defined as the number
    ///          of seconds elapsed since UNIX epoch.
    /// @remarks UNIX epoch = midnight (00:00:00) January 1, 1970 (UTC)
    time_t                                   /// @return integer part of seconds
    AsSystemTime(
        double* p_fractional_seconds = nullptr, ///< [out] fractional part of seconds
        int     fractional_digits = -1          ///< [in]  digits after decimal point in 'seconds' value (-1 = all significant digits)
        ) const;

    /// @name Date/Time
    //@{
    enum EDateTimeType {
        DATETIME_LOCAL = 0,
        DATETIME_UTC   = 1
    };
    /// @brief Returns the timestamp as a date/time string conforming to ISO 8601 extended format.
    /// @return date/time string in ISO 8601 format.
    /// @note  XML 'xs:dateTime' standard is compliant with ISO 8601.
    /// @example 2013-09-27T08:45:27.123+0200  (in 'local time' format, with TZD)
    /// @example 2013-09-27T08:45:27.123       (in 'local time' format, without TZD)
    /// @example 2013-09-27T06:45:27.123Z      (in 'UTC' format, with TZD)
    /// @example 2013-09-27T06:45:27.123       (in 'UTC' format, without TZD)
    std::string                             ///< [out] date/time string in ISO 8601 format
    AsIsoDateTime(
        EDateTimeType format,               ///< [in] 'local time' or 'UTC' format
        int           fractional_digits,    ///< [in] digits after decimal point in 'seconds' value
        bool          with_TZD = true       ///< [in] with/without 'Time Zone Designator'
        ) const;

    /// @brief Returns the timestamp as a LOCAL date/time string conforming to ISO 8601 extended format,
    ///        with a millisecond accuracy (default) and a 'Time Zone Designator' (default).
    std::string                         ///< [out] LOCAL date/time string in ISO 8601 format
    AsLocalDateTime(
        int  fractional_digits = 3,     ///< [in] digits after decimal point in 'seconds' value
        bool with_TZD = true            ///< [in] with/without 'Time Zone Designator'
        ) const {
        return AsIsoDateTime(DATETIME_LOCAL, fractional_digits, with_TZD);
    }
    /// @brief Returns the timestamp as an UTC date/time string conforming to ISO 8601 extended format,
    ///        with a millisecond accuracy (default) and a 'Time Zone Designator' (default).
    std::string                         ///< [out] UTC date/time string in ISO 8601 format
    AsUtcDateTime(
        int  fractional_digits = 3,     ///< [in] digits after decimal point in 'seconds' value
        bool with_TZD = true            ///< [in] with/without 'Time Zone Designator'
        ) const {
        return AsIsoDateTime(DATETIME_UTC, fractional_digits, with_TZD);
    }

#ifndef TIMESTAMP__NO_DEPRECATED
    /// @deprecated Please use AsIsoDateTime() instead.
    void
    AsLocalDateTime(
        std::string * p_date,           ///< [out] nullptr, or local date in "YYYY/MM/DD" format
        std::string * p_time,           ///< [out] nullptr, or local time in "HH:MM:SS.sss" format
        int           fractional_digits ///< [in] 'seconds' value: digits after decimal point
        ) const;
#endif
    //@}

    /// Renvoie la valeur interne du timestamp
    raw_type Raw() const noexcept {
        return this->counter_;
    }

    void operator += (TimeStamp const& other);
    void operator -= (TimeStamp const& other);
    void operator *= (int    value);
    void operator *= (double value);
    void operator /= (int    value);
    void operator /= (double value);
    void operator %= (TimeStamp const& other);

#ifndef TIMESTAMP__NO_STD_CHRONO_ITEROPERABILITY
    /// @name Interoperability with std::chrono::duration
    //@{
    template <typename REP_, typename PERIOD_>
    /* implicit */
    constexpr TimeStamp(std::chrono::duration<REP_, PERIOD_> d)
        : counter_{std::chrono::duration_cast<duration>(d).count()}
        {}

    template <typename REP_, typename PERIOD_>
    void Set(std::chrono::duration<REP_, PERIOD_> d) {
        counter_ = std::chrono::duration_cast<duration>(d).count();
    }

    template <typename REP_, typename PERIOD_>
    /* implicit */
    operator std::chrono::duration<REP_, PERIOD_> () const {
        return AsDuration<std::chrono::duration<REP_, PERIOD_>>();
    }

    template <typename STD_CHRONO_DURATION_>
    STD_CHRONO_DURATION_
    AsDuration() const {
        return std::chrono::duration_cast<STD_CHRONO_DURATION_>(duration{counter_});
    }
    //@}

    /// @name Interoperability with std::chrono::system_clock::time_point
    //@{
    /* implicit */
    constexpr TimeStamp(std::chrono::system_clock::time_point tp)
        // counter = duration since epoch (midnight (00:00:00), January 1, 1970 (UTC))
        : counter_{std::chrono::duration_cast<duration>(tp.time_since_epoch()).count()}
        {}

    /* implicit */
    operator std::chrono::system_clock::time_point () const {
        return AsTimePoint();
    }

    std::chrono::system_clock::time_point
    AsTimePoint() const {
        return std::chrono::system_clock::time_point{} + duration{counter_};
    }
    //@}
#endif
};

//------------------------------------------------------------------------------
inline TimeStamp
operator - (TimeStamp const& t) {
    t.AssertValid();
    return TimeStamp(- t.Raw(), TimeStamp::RAW);
}

inline TimeStamp
operator - (TimeStamp const& left, TimeStamp const& right) {
    left.AssertValid();
    right.AssertValid();
    return TimeStamp(left.Raw() - right.Raw(), TimeStamp::RAW);
}

inline TimeStamp
operator + (TimeStamp const& left, TimeStamp const& right) {
    left.AssertValid();
    right.AssertValid();
    return TimeStamp(left.Raw() + right.Raw(), TimeStamp::RAW);
}

inline TimeStamp
operator * (TimeStamp const& left, int right) {
    left.AssertValid();
    return TimeStamp(left.Raw() * right, TimeStamp::RAW);
}

inline TimeStamp
operator * (int left, TimeStamp const& right) {
    right.AssertValid();
    return TimeStamp(right.Raw() * left, TimeStamp::RAW);
}

inline TimeStamp
operator / (TimeStamp const& left, int right) {
    left.AssertValid();
    return TimeStamp(left.Raw() / right, TimeStamp::RAW);
}

inline TimeStamp
operator * (TimeStamp const& left, double right) {
    left.AssertValid();
    return TimeStamp(
        static_cast<TimeStamp::raw_type>(left.Raw() * right),
        TimeStamp::RAW
        );
}

inline TimeStamp
operator * (double left, TimeStamp const& right) {
    right.AssertValid();
    return TimeStamp(
        static_cast<TimeStamp::raw_type>(right.Raw() * left),
        TimeStamp::RAW
        );
}

inline TimeStamp
operator / (TimeStamp const& left, double right) {
    left.AssertValid();
    return TimeStamp(
        static_cast<TimeStamp::raw_type>(left.Raw() / right),
        TimeStamp::RAW
        );
}

inline double
operator / (TimeStamp const& left, TimeStamp const& right) {
    left.AssertValid();
    right.AssertValid();
    return static_cast<double>(left.Raw())
         / static_cast<double>(right.Raw());
}

inline TimeStamp
operator % (TimeStamp const& left, TimeStamp const& right) {
    left.AssertValid();
    right.AssertValid();
    return TimeStamp(left.Raw() % right.Raw(), TimeStamp::RAW);
}

inline void
TimeStamp::operator += (TimeStamp const& other) {
    AssertValid();
    other.AssertValid();
    counter_ += other.counter_;
}

inline void
TimeStamp::operator -= (TimeStamp const& other) {
    AssertValid();
    other.AssertValid();
    counter_ -= other.counter_;
}

inline void
TimeStamp::operator *= (int value) {
    AssertValid();
    counter_ *= value;
}

inline void
TimeStamp::operator *= (double value) {
    AssertValid();
    counter_ = static_cast<TimeStamp::raw_type>(counter_ * value);
}

inline void
TimeStamp::operator /= (int value) {
    AssertValid();
    counter_ /= value;
}

inline void
TimeStamp::operator /= (double value) {
    AssertValid();
    counter_ = static_cast<TimeStamp::raw_type>(counter_ / value);
}

inline void
TimeStamp::operator %= (TimeStamp const& other) {
    AssertValid();
    other.AssertValid();
    counter_ %= other.counter_;
}

//------------------------------------------------------------------------------
#define DEF_TIMESTAMP_BINARY_BOOLEAN_OP_(op)                       \
    inline bool                                                    \
    operator op (TimeStamp const& left, TimeStamp const& right) {  \
        left.AssertValid();                                        \
        right.AssertValid();                                       \
        return left.Raw() op right.Raw();                          \
    }

    DEF_TIMESTAMP_BINARY_BOOLEAN_OP_(==)
    DEF_TIMESTAMP_BINARY_BOOLEAN_OP_(!=)
    DEF_TIMESTAMP_BINARY_BOOLEAN_OP_(<)
    DEF_TIMESTAMP_BINARY_BOOLEAN_OP_(>)
    DEF_TIMESTAMP_BINARY_BOOLEAN_OP_(<=)
    DEF_TIMESTAMP_BINARY_BOOLEAN_OP_(>=)
#undef DEF_TIMESTAMP_BINARY_BOOLEAN_OP_

//------------------------------------------------------------------------------
inline bool
TimeStamp::IsValid() const noexcept {
    return counter_ != INVALID().counter_;
}

//------------------------------------------------------------------------------
/// Absolute value of a TimeStamp
inline TimeStamp
Abs(TimeStamp const& ts) {
    ts.AssertValid();
    return TimeStamp(ts.Raw() >= 0 ? ts.Raw() : -ts.Raw() , TimeStamp::RAW);
}

//------------------------------------------------------------------------------
/// Insert a valid timestamp in an output text stream as a floating-point value
/// in seconds, with 3 decimal places (i.e. millisecond accuracy).
/// Insert an invalid timestamp as string "<invalid>".
std::ostream & operator << (std::ostream & r_out, TimeStamp const& timestamp);
std::string ToString(TimeStamp const& timestamp);

//------------------------------------------------------------------------------
/// Try to read a timestamp from an input text stream, assuming it is a
/// floating point value representing a duration in seconds.
/// If the reading fails, the timestamp is invalid.
std::istream& operator >> (std::istream & r_in, TimeStamp& r_timestamp);

//------------------------------------------------------------------------------
/// Specialize std::hash<> for timestamps
namespace std {
    template <>
    struct hash<TimeStamp>
    {
        std::size_t operator()(TimeStamp const& ts) const {
            return hash<TimeStamp::raw_type>{}(ts.Raw());
        }
    };
}
