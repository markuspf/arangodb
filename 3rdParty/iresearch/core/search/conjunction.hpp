//
// IResearch search engine 
// 
// Copyright (c) 2016 by EMC Corporation, All Rights Reserved
// 
// This software contains the intellectual property of EMC Corporation or is licensed to
// EMC Corporation from third parties. Use of this software and the intellectual property
// contained therein is expressly limited to the terms and conditions of the License
// Agreement under which it is provided by or on behalf of EMC.
// 

#ifndef IRESEARCH_CONJUNCTION_H
#define IRESEARCH_CONJUNCTION_H

#include "cost.hpp"
#include "score_doc_iterators.hpp"
#include "analysis/token_attributes.hpp"
#include "utils/type_limits.hpp"

NS_ROOT

////////////////////////////////////////////////////////////////////////////////
/// @class score_iterator_adapter
/// @brief adapter to use doc_iterator with conjunction and disjunction
////////////////////////////////////////////////////////////////////////////////
struct score_iterator_adapter : util::noncopyable {
  score_iterator_adapter(doc_iterator::ptr&& it) NOEXCEPT
    : it(std::move(it)) {
    score = &irs::score::extract(this->it->attributes());
  }

  score_iterator_adapter(score_iterator_adapter&& rhs) NOEXCEPT
    : it(std::move(rhs.it)), score(rhs.score) {
  }

  score_iterator_adapter& operator=(score_iterator_adapter&& rhs) NOEXCEPT {
    if (this != &rhs) {
      it = std::move(rhs.it);
      score = rhs.score;
    }
    return *this;
  }


  doc_iterator* operator->() const NOEXCEPT {
    return it.get();
  }

  operator doc_iterator::ptr&() NOEXCEPT {
    return it;
  }

  doc_iterator::ptr it;
  const irs::score* score;
}; // score_iterator_adapter

////////////////////////////////////////////////////////////////////////////////
/// @class conjunction
///-----------------------------------------------------------------------------
/// c |  [0] <-- lead (the least cost iterator)
/// o |  [1]    |
/// s |  [2]    | tail (other iterators)
/// t |  ...    |
///   V  [n] <-- end
///-----------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
class conjunction : public doc_iterator_base {
 public:
  typedef score_iterator_adapter doc_iterator_t;
  typedef std::vector<doc_iterator_t> doc_iterators_t;
  typedef doc_iterators_t::const_iterator iterator;

  conjunction(
      doc_iterators_t&& itrs,
      const order::prepared& ord = order::prepared::unordered())
    : doc_iterator_base(ord),
      itrs_(std::move(itrs)) {
    assert(!itrs_.empty());

    // sort subnodes in ascending order by their cost
    std::sort(itrs_.begin(), itrs_.end(),
      [](const doc_iterator_t& lhs, const doc_iterator_t& rhs) {
        return cost::extract(lhs->attributes(), cost::MAX) < cost::extract(rhs->attributes(), cost::MAX);
    });

    // set front iterator
    front_ = itrs_.front().it.get();

    // estimate iterator (front's cost is already cached)
    estimate(cost::extract(front_->attributes(), cost::MAX));

    // prepare score
    prepare_score([this](byte_type* score) {
      ord_->prepare_score(score);
      for (auto& it : itrs_) {
        const auto* it_score = it.score;
        it_score->evaluate();
        ord_->add(score, it_score->c_str());
      }
    });
  }

  iterator begin() const { return itrs_.begin(); }
  iterator end() const { return itrs_.end(); }

  // size of conjunction
  size_t size() const { return itrs_.size(); }

  virtual doc_id_t value() const override {
    return front_->value();
  }

  virtual bool next() override {
    if (!front_->next()) {
      return false;
    }

    return !type_limits<type_t::doc_id_t>::eof(converge(front_->value()));
  }

  virtual doc_id_t seek(doc_id_t target) override {
    if (type_limits<type_t::doc_id_t>::eof(target = front_->seek(target))) {
      return target;
    }

    return converge(target);
  }

 private:
  // tries to converge front_ and other iterators to the specified target.
  // if it impossible tries to find first convergence place
  doc_id_t converge(doc_id_t target) {
    for (auto rest = seek_rest(target); target != rest;) {
      target = front_->seek(rest);
      rest = seek_rest(target);
    }

    return target;
  }

  // seeks all iterators except the
  // first to the specified target
  doc_id_t seek_rest(doc_id_t target) {
    if (type_limits<type_t::doc_id_t>::eof(target)) {
      return target;
    }

    for (auto it = itrs_.begin()+1, end = itrs_.end(); it != end; ++it) {
      const auto doc = (*it)->seek(target);

      if (target < doc) {
        return doc;
      }
    }

    return target;
  }

  doc_iterators_t itrs_;
  irs::doc_iterator* front_;
}; // conjunction

//////////////////////////////////////////////////////////////////////////////
/// @returns conjunction iterator created from the specified sub iterators 
//////////////////////////////////////////////////////////////////////////////
template<typename Conjunction, typename... Args>
doc_iterator::ptr make_conjunction(
    typename Conjunction::doc_iterators_t&& itrs,
    Args&&... args) {
  switch (itrs.size()) {
    case 0:
      // empty or unreachable search criteria
      return doc_iterator::empty();
    case 1:
      // single sub-query
      return std::move(itrs.front());
  }

  // conjunction
  return doc_iterator::make<Conjunction>(
      std::move(itrs), std::forward<Args>(args)...
  );
}

NS_END // ROOT

#endif // IRESEARCH_CONJUNCTION_H
