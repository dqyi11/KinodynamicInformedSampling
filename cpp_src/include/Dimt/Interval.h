#ifndef INTERVAL_H
#define INTERVAL_H

#include <limits>
#include <list>

class Interval
{
public:
    Interval(double min, double max);
    Interval();
    Interval(double val, bool is_val_lower);

    double get_min() const;
    double get_max() const;

    void set_min(double min);
    void set_max(double max);
    void set_min_unbounded();
    void set_max_unbounded();

    bool is_unbounded() const;
    bool is_bounded() const;
    bool is_lower_bounded() const;
    bool is_higher_bounded() const;

    void print_nice();

private:
    double _min, _max;
};  // Interval

class IntervalSet
{
public:
    typedef std::list<Interval> IntervalList;
    typedef typename IntervalList::iterator IntervalListIter;

public:
    IntervalSet(bool all_is_legal = true);
    void insert_and(const Interval &i);
    void insert_and_complement(const Interval &i);

    bool is_unbounded() const;

    void print_nice();

    IntervalList &get_intervals();
    const IntervalList &get_intervals() const;

    void assign_interval_list(const IntervalList &new_intervals);
    template <typename InputIterator>
    void assign_interval_list(InputIterator begin, InputIterator end);

private:
    void insert_or_lower_unbounded(const Interval &i);
    void insert_or_higher_unbounded(const Interval &i);
    void insert_or_bounded(const Interval &i);
    void insert_and_lower_unbounded(const Interval &i);
    void insert_and_higher_unbounded(const Interval &i);
    void insert_and_bounded(const Interval &i);

private:
    // every interval is a legal partioining of the line
    IntervalList intervals;
};  // IntervalSet

#endif  // INTERVAL_H
