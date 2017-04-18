#include "Dimt/Interval.h"
#include <iostream>

Interval::Interval(double min, double max) :_min(min), _max(max)
{}

Interval::Interval() :_min(std::numeric_limits<double>::min()), _max(std::numeric_limits<double>::max())
{}

Interval::Interval(double val, bool is_val_lower)
{
	if (is_val_lower)
	{
		_min = val;
		_max = std::numeric_limits<double>::max();
	}
	else // is_val_lower = false
	{
		_min = std::numeric_limits<double>::min();
		_max = val;
	}
}

double Interval::get_min() const
{
	return _min;
}

double Interval::get_max() const
{
	return _max;
}

void Interval::set_min(double min)
{
	_min = min;
	return;
}

void Interval::set_max(double max)
{
	_max = max;
	return;
}

void Interval::set_min_unbounded()
{
	_min = std::numeric_limits<double>::min();
}

void Interval::set_max_unbounded()
{
	_max = std::numeric_limits<double>::max();
}

bool Interval::is_unbounded() const
{
	if ((_min == std::numeric_limits<double>::min()) &&
		(_max == std::numeric_limits<double>::max()))
		return true;
	return false;
}

bool Interval::is_bounded() const
{
	if ((_min != std::numeric_limits<double>::min()) &&
		(_max != std::numeric_limits<double>::max()))
		return true;
	return false;
}

bool Interval::is_lower_bounded() const
{
	return (_min != std::numeric_limits<double>::min());
}

bool Interval::is_higher_bounded() const
{
	return (_max != std::numeric_limits<double>::max());
}

void Interval::print_nice()
{
	std::cout << "[ ";
	if (is_lower_bounded() == false)
		std::cout << "-oo";
	else
		std::cout << _min;

	std::cout << " --> ";

	if (is_higher_bounded() == false)
		std::cout << "oo";
	else
		std::cout << _max;

	std::cout << " ]   ";
}

IntervalSet::IntervalSet(bool all_is_legal )
{
	if (all_is_legal)
	{
		intervals.push_front(Interval());
	}
}

void IntervalSet::insert_and(const Interval& i)
{
	if (is_unbounded())
	{
		intervals.clear();
		intervals.push_front(i);
		return;
	}
	if (intervals.empty())
		return;

	if (i.is_lower_bounded() == false)
	{
		insert_and_lower_unbounded(i);
		return;
	}
	if (i.is_higher_bounded() == false)
	{
		insert_and_higher_unbounded(i);
		return;
	}
	//i is unbounded
	insert_and_bounded(i);
	return;
}

void IntervalSet::insert_and_complement(const Interval& i)
{

	Interval i1(i.get_min(), false);
	Interval i2(i.get_max(), true);
	IntervalSet tmp_intervals1(true);
	IntervalSet tmp_intervals2(true);

	tmp_intervals1.assign_interval_list(intervals.begin(), intervals.end());
	tmp_intervals2.assign_interval_list(intervals.begin(), intervals.end());

	tmp_intervals1.insert_and(i1);
	tmp_intervals2.insert_and(i2);

	intervals.clear();
	intervals.assign(tmp_intervals1.intervals.begin(), tmp_intervals1.intervals.end());
	intervals.insert(intervals.end(), tmp_intervals2.intervals.begin(), tmp_intervals2.intervals.end());

	return;
}

bool IntervalSet::is_unbounded() const
{
	if (intervals.size() == 1)
		return intervals.front().is_unbounded();
	return false;
}

void IntervalSet::print_nice()
{
	IntervalListIter it;
	std::cout << " { ";
	for (it = intervals.begin(); it != intervals.end(); )
	{
		it->print_nice();
		++it;
		if (it != intervals.end())
			std::cout << " , ";
	}
	std::cout << " } " << std::endl;
	return;
}

IntervalSet::IntervalList& IntervalSet::get_intervals()
{
	return intervals;
}

const IntervalSet::IntervalList& IntervalSet::get_intervals() const
{
	return intervals;
}

void IntervalSet::assign_interval_list(const IntervalList& new_intervals)
{
	intervals.assign(new_intervals.begin(), new_intervals.end());
	return;
}

template <typename InputIterator>
void IntervalSet::assign_interval_list(InputIterator begin, InputIterator end)
{
	intervals.assign(begin, end);
	return;
}

void IntervalSet::insert_or_lower_unbounded(const Interval& i)
{
	if (intervals.empty())
	{
		intervals.push_front(i);
		return;
	}
	if (intervals.front().is_lower_bounded() == false)
	{
		if (i.get_max() < intervals.front().get_max())
			return;
		else
		{	//first element has no effect, remove it
			intervals.pop_front();
			if (intervals.empty())
			{
				intervals.push_front(i);
				return;
			}
		}
	}

	if (intervals.back().is_higher_bounded() == false)
	{
		if (i.get_max() > intervals.back().get_min())
		{
			intervals.clear();
			intervals.push_back(Interval());
			return;
		}
	}

	//at this point the first elemt is bounded 
	//and if the last is unbounded we will not
	//reach it
	for (IntervalListIter it(intervals.begin()); it != intervals.end(); ++it)
	{
		if (it->get_min() > i.get_max())
		{
			intervals.erase(intervals.begin(), it);
			intervals.push_front(i);
			return;
		}

		if (it->get_max() > i.get_max())
		{
			Interval tmp(i.get_max(), false);
			++it;
			intervals.erase(intervals.begin(), it);
			intervals.push_front(tmp);
			return;
		}
	}

	//if we reached here then the interval should be i
	intervals.clear();
	intervals.push_front(i);
	return;
}

void IntervalSet::insert_or_higher_unbounded(const Interval& i)
{
	if (intervals.empty())
	{
		intervals.push_front(i);
		return;
	}
	if (intervals.front().is_lower_bounded() == false)
	{
		if (i.get_min() < intervals.front().get_max())
		{
			intervals.clear();
			intervals.push_back(Interval());
			return;
		}
	}
	if (intervals.back().is_higher_bounded() == false)
	{
		if (i.get_min() > intervals.back().get_min())
			return;
		else
		{
			//lastelement has no effect, remove it
			intervals.pop_back();
			if (intervals.empty())
			{
				intervals.push_front(i);
				return;
			}
		}
	}
	//at this point if the first elemt is unbounded it has no effect
	//and the last is unbounded
	IntervalListIter it = intervals.end();
	for (; it != intervals.begin(); )
	{
		--it;
		if (it->get_max() < i.get_min())
		{
			++it;
			intervals.erase(it, intervals.end());
			intervals.push_back(i);
			return;
		}

		if (it->get_min() < i.get_min())
		{
			Interval tmp(it->get_min(), true);
			intervals.erase(it, intervals.end());
			intervals.push_back(tmp);
			return;
		}
	}
	//if we reached here then the interval should be i
	intervals.clear();
	intervals.push_front(i);
	return;
}

void IntervalSet::insert_or_bounded(const Interval& i)
{

	Interval tmp1(i.get_min(), false);
	Interval tmp2(i.get_max(), true);

	IntervalSet  tmp_IntervalSet_1(true);
	IntervalList tmp_interval_list1 = tmp_IntervalSet_1.get_intervals();

	IntervalSet  tmp_IntervalSet_2(true);
	IntervalList tmp_interval_list2 = tmp_IntervalSet_2.get_intervals();

	tmp_interval_list1.assign(intervals.begin(), intervals.end());
	tmp_interval_list2.assign(intervals.begin(), intervals.end());

	tmp_IntervalSet_1.insert_and_lower_unbounded(tmp1);
	tmp_IntervalSet_2.insert_and_lower_unbounded(tmp2);
	if (tmp_interval_list1.empty())
	{
		tmp_interval_list1.push_back(i);
	}
	else
	{
		if (tmp_interval_list1.back().get_max() == i.get_min())
			tmp_interval_list1.back().set_max(i.get_max());
		else
			tmp_interval_list1.push_back(i);
	}

	if (tmp_interval_list2.empty())
	{
		intervals.assign(tmp_interval_list1.begin(), tmp_interval_list1.end());
	}
	else
	{
		if (tmp_interval_list2.front().get_min() == tmp_interval_list1.back().get_max())
		{
			//we need to unify the last element of 
			//tmp_intervals1 with the first of tmp_intervals2
			if (tmp_interval_list1.back().is_lower_bounded() == false)
			{
				tmp_interval_list2.front().set_min_unbounded();
				intervals.clear();
				intervals.insert(intervals.begin(), tmp_interval_list2.begin(), tmp_interval_list2.end());
				return;
			}
			else if (tmp_interval_list2.front().is_higher_bounded() == false)
			{
				tmp_interval_list1.back().set_max_unbounded();
				intervals.assign(tmp_interval_list1.begin(), tmp_interval_list1.end());
				return;
			}
			else
			{
				tmp_interval_list1.back().set_max(tmp_interval_list2.front().get_max());
				tmp_interval_list2.pop_front();
				intervals.assign(tmp_interval_list1.begin(), tmp_interval_list1.end());
				intervals.insert(intervals.end(), tmp_interval_list2.begin(), tmp_interval_list2.end());
				return;
			}
		}
		else
		{
			intervals.assign(tmp_interval_list1.begin(), tmp_interval_list1.end());
			intervals.insert(intervals.end(), tmp_interval_list2.begin(), tmp_interval_list2.end());
		}
	}
	return;
}

void IntervalSet::insert_and_lower_unbounded(const Interval& i)
{
	if (intervals.empty())
		return;
	if (intervals.back().is_higher_bounded() == false)
	{
		//highest value can be i.highest
		if (i.get_max() < intervals.back().get_min())
		{
			intervals.pop_back();
			if (intervals.empty())
				return;
		}
		else
		{
			Interval tmp(intervals.back().get_min(), i.get_max());
			intervals.pop_back();
			intervals.push_back(tmp);
		}
	}
	if (intervals.front().is_lower_bounded() == false)
	{
		if (i.get_max() < intervals.front().get_max())
		{
			intervals.clear();
			intervals.push_front(i);
			return;
		}
	}
	else
	{
		//just take care of first element for the loop
		if (intervals.front().get_min() > i.get_max())
		{
			intervals.clear();
			return;
		}
		if (intervals.front().get_max() > i.get_max())
		{
			Interval tmp(intervals.front().get_min(), i.get_max());
			intervals.clear();
			intervals.push_front(tmp);
			return;
		}
	}
	IntervalListIter it = intervals.begin();
	++it;
	for (; it != intervals.end(); ++it)
	{
		if (it->get_min() >= i.get_max())
		{
			intervals.erase(it, intervals.end());
			return;
		}

		if (it->get_max() > i.get_max())
		{
			Interval tmp(it->get_min(), i.get_max());
			intervals.erase(it, intervals.end());
			intervals.push_back(tmp);
			return;
		}
	}
	//intervals.clear();
	return;
}

void IntervalSet::insert_and_higher_unbounded(const Interval& i)
{
	if (intervals.empty())
		return;
	if (intervals.back().is_higher_bounded() == false)
	{
		if (i.get_min() > intervals.back().get_min())
		{
			intervals.clear();
			intervals.push_front(i);
			return;
		}
	}
	if (intervals.front().is_lower_bounded() == false)
	{
		if (i.get_min() > intervals.front().get_max())
		{
			intervals.pop_front();
			if (intervals.empty())
				return;
		}
		else
		{
			Interval tmp(i.get_min(), intervals.front().get_max());
			intervals.clear();
			intervals.push_front(tmp);
			return;
		}
	}
	else
	{
		//just take care of first element for the loop
		if (intervals.front().get_min() > i.get_min())
		{
			return;
		}

		if (intervals.front().get_max() > i.get_min())
		{
			Interval tmp(i.get_min(), intervals.front().get_max());
			intervals.pop_front();
			intervals.push_front(tmp);
			return;
		}
	}
	IntervalListIter it = intervals.begin();
	//++it;
	for (; it != intervals.end(); ++it)
	{
		if (it->get_min() > i.get_min())
		{
			intervals.erase(intervals.begin(), it);
			return;
		}
		if (it->get_max() > i.get_min())
		{
			Interval tmp(i.get_min(), it->get_max());
			++it;
			intervals.erase(intervals.begin(), it);
			intervals.push_front(tmp);
			return;
		}
	}
	intervals.clear();
	return;
}
void IntervalSet::insert_and_bounded(const Interval& i)
{
	Interval tmp1(i.get_max(), false);
	Interval tmp2(i.get_min(), true);

	insert_and_lower_unbounded(tmp1);
	insert_and_higher_unbounded(tmp2);
	return;
}
