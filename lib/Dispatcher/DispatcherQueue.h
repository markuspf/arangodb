////////////////////////////////////////////////////////////////////////////////
/// @brief dispatcher queue
///
/// @file
///
/// DISCLAIMER
///
/// Copyright 2014 ArangoDB GmbH, Cologne, Germany
/// Copyright 2004-2014 triAGENS GmbH, Cologne, Germany
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///
///     http://www.apache.org/licenses/LICENSE-2.0
///
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// Copyright holder is ArangoDB GmbH, Cologne, Germany
///
/// @author Dr. Frank Celler
/// @author Martin Schoenert
/// @author Copyright 2014, ArangoDB GmbH, Cologne, Germany
/// @author Copyright 2009-2014, triAGENS GmbH, Cologne, Germany
////////////////////////////////////////////////////////////////////////////////

#ifndef ARANGODB_DISPATCHER_DISPATCHER_QUEUE_H
#define ARANGODB_DISPATCHER_DISPATCHER_QUEUE_H 1

#include "Basics/Common.h"

#include "Basics/ConditionVariable.h"
#include "Dispatcher/Dispatcher.h"

// -----------------------------------------------------------------------------
// --SECTION--                                              forward declarations
// -----------------------------------------------------------------------------

namespace triagens {
  namespace rest {
    class DispatcherThread;
    class Job;

// -----------------------------------------------------------------------------
// --SECTION--                                             class DispatcherQueue
// -----------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/// @brief dispatcher queue
////////////////////////////////////////////////////////////////////////////////

    class DispatcherQueue {
      friend class Dispatcher;
      friend class DispatcherThread;

      private:
        DispatcherQueue (DispatcherQueue const&);
        DispatcherQueue& operator= (DispatcherQueue const&);

// -----------------------------------------------------------------------------
// --SECTION--                                      constructors and destructors
// -----------------------------------------------------------------------------

      public:

////////////////////////////////////////////////////////////////////////////////
/// @brief constructs a new dispatcher queue
////////////////////////////////////////////////////////////////////////////////

        DispatcherQueue (Scheduler*,
                         Dispatcher*,
                         std::string const& name,
                         Dispatcher::newDispatcherThread_fptr,
                         void* threadData,
                         size_t nrThreads,
                         size_t maxSize);

////////////////////////////////////////////////////////////////////////////////
/// @brief destructor
////////////////////////////////////////////////////////////////////////////////

        ~DispatcherQueue ();

// -----------------------------------------------------------------------------
// --SECTION--                                                    public methods
// -----------------------------------------------------------------------------

      public:

////////////////////////////////////////////////////////////////////////////////
/// @brief adds a job
////////////////////////////////////////////////////////////////////////////////

        bool addJob (Job*);

////////////////////////////////////////////////////////////////////////////////
/// @brief tries to cancel a job
////////////////////////////////////////////////////////////////////////////////

        bool cancelJob (uint64_t);

////////////////////////////////////////////////////////////////////////////////
/// @brief indicates that thread is doing a blocking operation
////////////////////////////////////////////////////////////////////////////////

        void blockThread (DispatcherThread* thread);

////////////////////////////////////////////////////////////////////////////////
/// @brief indicates that thread has resumed work
////////////////////////////////////////////////////////////////////////////////

        void unblockThread (DispatcherThread* thread);

////////////////////////////////////////////////////////////////////////////////
/// @brief starts a queue
////////////////////////////////////////////////////////////////////////////////

        bool start ();

////////////////////////////////////////////////////////////////////////////////
/// @brief begins the shutdown sequence the queue
////////////////////////////////////////////////////////////////////////////////

        void beginShutdown ();

////////////////////////////////////////////////////////////////////////////////
/// @brief shut downs the queue
////////////////////////////////////////////////////////////////////////////////

        void shutdown ();

////////////////////////////////////////////////////////////////////////////////
/// @brief checks if the dispatcher queues are up and running
////////////////////////////////////////////////////////////////////////////////

        bool isStarted ();

////////////////////////////////////////////////////////////////////////////////
/// @brief is the queue still active
////////////////////////////////////////////////////////////////////////////////

        bool isRunning ();

////////////////////////////////////////////////////////////////////////////////
/// @brief starts a new queue thread
////////////////////////////////////////////////////////////////////////////////

        bool startQueueThread ();

////////////////////////////////////////////////////////////////////////////////
/// @brief return name
////////////////////////////////////////////////////////////////////////////////

        std::string name () const {
          return _name;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief sets the process affinity
////////////////////////////////////////////////////////////////////////////////

        void setProcessorAffinity (const std::vector<size_t>& cores);

// -----------------------------------------------------------------------------
// --SECTION--                                                 private variables
// -----------------------------------------------------------------------------

      private:

////////////////////////////////////////////////////////////////////////////////
/// @brief name
////////////////////////////////////////////////////////////////////////////////

        std::string const _name;

////////////////////////////////////////////////////////////////////////////////
/// @brief thread data
////////////////////////////////////////////////////////////////////////////////

        void* _threadData;

////////////////////////////////////////////////////////////////////////////////
/// @brief monopolistic jobs
////////////////////////////////////////////////////////////////////////////////

        basics::ConditionVariable _accessQueue;

////////////////////////////////////////////////////////////////////////////////
/// @brief list of ready jobs
////////////////////////////////////////////////////////////////////////////////

        std::list<Job*> _readyJobs;

////////////////////////////////////////////////////////////////////////////////
/// @brief current running job
////////////////////////////////////////////////////////////////////////////////

        std::set<Job*> _runningJobs;

////////////////////////////////////////////////////////////////////////////////
/// @brief maximum queue size (number of jobs)
////////////////////////////////////////////////////////////////////////////////

        size_t _maxSize;

////////////////////////////////////////////////////////////////////////////////
/// @brief queue is shutting down
////////////////////////////////////////////////////////////////////////////////

        volatile sig_atomic_t _stopping;

////////////////////////////////////////////////////////////////////////////////
/// @brief list of started threads
////////////////////////////////////////////////////////////////////////////////

        std::set<DispatcherThread*> _startedThreads;

////////////////////////////////////////////////////////////////////////////////
/// @brief list of stopped threads
////////////////////////////////////////////////////////////////////////////////

        std::list<DispatcherThread*> _stoppedThreads;

////////////////////////////////////////////////////////////////////////////////
/// @brief number of started jobs
///
/// Whenever a new thread is started, this number is increased by 1. As soon as
/// the thread enters its main event loop, this number is decreased by 1 and
/// the number running threads _nrRunning is increased by 1.
///
/// Therefore _nrStarted and _nrRunning is the number of threads which are or
/// soon be available for the dispatcher queue.
////////////////////////////////////////////////////////////////////////////////

        size_t _nrStarted;

////////////////////////////////////////////////////////////////////////////////
/// @brief number of threads that are up
///
/// As soon as a thread enters its main event loop, this number is increased by
/// 1. As soon as the thread leaves its main event loop, this number is
/// decreased by 1.
///
/// This indicates the current number of threads currently within their main
/// event loop. The difference to _nrRunning is, that _nrRunning is decreased by
/// 1, if a thread enters a special job.
////////////////////////////////////////////////////////////////////////////////

        size_t _nrUp;

////////////////////////////////////////////////////////////////////////////////
/// @brief number of running jobs
///
/// As soon as a thread enters its main event loop, this number is increased by
/// one. As soon as the thread leaves its main event loop or works on a special
/// job, this number is decreaes by 1.
////////////////////////////////////////////////////////////////////////////////

        size_t _nrRunning;

////////////////////////////////////////////////////////////////////////////////
/// @brief number of waiting jobs
///
/// Whenever a threads waits for more work, this number is increased by 1.  As
/// soon as the thread leaves the wait because it received a broadcast, this
/// number is decreased by 1.
////////////////////////////////////////////////////////////////////////////////

        size_t _nrWaiting;

////////////////////////////////////////////////////////////////////////////////
/// @brief number of stopped jobs
///
/// As soon as a thread leaves its main event loop, this number is increased by
/// 1 and the thread is put onto a cleanup list. Eventually all threads on that
/// list are deleted and the number is reset to zero.
////////////////////////////////////////////////////////////////////////////////

        size_t _nrStopped;

////////////////////////////////////////////////////////////////////////////////
/// @brief number of blocked threads
///
/// The number of threads, that are blocked for some reason. 
////////////////////////////////////////////////////////////////////////////////

        size_t _nrBlocked;

////////////////////////////////////////////////////////////////////////////////
/// @brief total number of threads
///
/// This number is fixed. It is the number of pre-configured threads from the
/// configuration file.
///
/// The following hold, after startup and before shutdown:
///
/// _nrRunning + _nrStarted = _nrThreads
/// _nrRunning + _nrSpecial + _nrWaiting = _nrUp
////////////////////////////////////////////////////////////////////////////////

        size_t _nrThreads;

////////////////////////////////////////////////////////////////////////////////
/// @brief last time we created or deleted a queue thread
////////////////////////////////////////////////////////////////////////////////

        double _lastChanged;

////////////////////////////////////////////////////////////////////////////////
/// @brief grace period before shuting down queue threads
////////////////////////////////////////////////////////////////////////////////

        double _gracePeriod;

////////////////////////////////////////////////////////////////////////////////
/// @brief scheduler
////////////////////////////////////////////////////////////////////////////////

        Scheduler* _scheduler;

////////////////////////////////////////////////////////////////////////////////
/// @brief dispatcher
////////////////////////////////////////////////////////////////////////////////

        Dispatcher* _dispatcher;

////////////////////////////////////////////////////////////////////////////////
/// @brief thread creator function
////////////////////////////////////////////////////////////////////////////////

        Dispatcher::newDispatcherThread_fptr createDispatcherThread;

////////////////////////////////////////////////////////////////////////////////
/// @brief cores to use for affinity
////////////////////////////////////////////////////////////////////////////////

        std::vector<size_t> _affinityCores;

////////////////////////////////////////////////////////////////////////////////
/// @brief next affinity core to use
////////////////////////////////////////////////////////////////////////////////

        size_t _affinityPos;
    };
  }
}

#endif

// -----------------------------------------------------------------------------
// --SECTION--                                                       END-OF-FILE
// -----------------------------------------------------------------------------

// Local Variables:
// mode: outline-minor
// outline-regexp: "/// @brief\\|/// {@inheritDoc}\\|/// @page\\|// --SECTION--\\|/// @\\}"
// End:
