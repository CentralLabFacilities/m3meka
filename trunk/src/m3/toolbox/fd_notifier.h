/*
 * Notifies when ever data is available on a file descriptor.
 * Copyright (C) 2011  Meka Robotics
 * Author <pierrelucbacon@mekabot.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef M3FDNOTIFIER_H_
#define M3FDNOTIFIER_H_

#include <vector>
#include <stdexcept>
#include <pthread.h>

/**
 * Inherit from this class in order to
 * be notified by the FdNotifier class
 */
class M3FdNotifierObserver {
public:
	M3FdNotifierObserver() {};
	virtual ~M3FdNotifierObserver() {};

	/**
	 * @param fd The file descriptor on which data is available
	 */
    virtual void onDataIn(int fd) {};

	/**
	 * @param fd The file descriptor on which data is available
	 */
	virtual void onDataOut(int fd) {};
};

/**
 * Notifies when ever data is available on a file descriptor.
 */
class M3FdNotifier {
public:
	M3FdNotifier(int fd);
	virtual ~M3FdNotifier();

	/**
	 * @throw std::logic_error if the M3FdNotifier thread is already running.
	 */
	void start() throw (std::logic_error);

	/**
	 * @throw std::logic_error if the M3FdNotifier thread is not running.
	 */
	void stop() throw (std::logic_error);
	
	/**
	 * Block until the notifier thread exits
	 */
	void join() throw (std::logic_error);

	/**
	 * @param observer The observer to subscribe
	 */
	void subscribe(M3FdNotifierObserver* observer);

	/**
	 * @param observer The observer to unsubscribe
	 */
	void unsubscribe(M3FdNotifierObserver* observer);

protected:
	/**
	 * Block on the file descriptor until there is
	 * data to be read.
	 * @throw std::runtime_error If select() fails on the port.
	 */
	void waitDataIn() throw (std::runtime_error);

	/**
	 * Block on the file descriptor until there is
	 * data to be read.
	 * @throw std::runtime_error If select() fails on the port.
	 */
	void waitDataOut() throw (std::runtime_error);

	/**
	 * The main loop for checkint the input port.
	 * @param data Data to be passed
	 */
	static void checkInput(void* data);

	/**
	 * The main loop for checking the output port.
	 * @param data Data to be passed
	 */
	static void checkOutput(void* data);

	/**
	 * Notify all the subscribed observers that data is available
	 * for read().
	 */
	void notifyRead(int fd);

	/**
	 * Notify all the subscribed observers that data can be now
	 * written.
	 */
	void notifyWrite(int fd);

private:
	int fd;
	
	struct ThreadData
	{
	    int threadNumber;
	    M3FdNotifier* self;
	    pthread_mutex_t mutex1;
	};
	ThreadData threadData;

	pthread_t readThread;
	pthread_t writeThread;

	std::vector<M3FdNotifierObserver*> observers;
	typedef std::vector<M3FdNotifierObserver*>::iterator M3FdNotifierObserversIt;
};

#endif /* M3FDNOTIFIER_H_ */
