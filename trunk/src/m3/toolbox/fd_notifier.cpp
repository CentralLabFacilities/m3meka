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
#include "fd_notifier.h"

#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <algorithm>

static pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

M3FdNotifier::M3FdNotifier(int fd) :
	fd(fd)  {
}

M3FdNotifier::~M3FdNotifier() {
	// TODO Auto-generated destructor stub
}

void M3FdNotifier::unsubscribe(M3FdNotifierObserver *observer) {
	M3FdNotifierObserversIt it;
	it = std::find(observers.begin(), observers.end(), observer);
	if (it != observers.end()) {
		observers.erase(it);
	}
}

void M3FdNotifier::subscribe(M3FdNotifierObserver *observer) {
	observers.push_back(observer);
}

void M3FdNotifier::notifyRead(int fd) {
	M3FdNotifierObserversIt it;
	for (it = observers.begin(); it != observers.end(); it++) {
		(*it)->onDataIn(fd);
	}
}

void M3FdNotifier::notifyWrite(int fd) {
	M3FdNotifierObserversIt it;
	for (it = observers.begin(); it != observers.end(); it++) {
		(*it)->onDataOut(fd);
	}
}

void M3FdNotifier::checkInput(void* data) {
	ThreadData* thData = (ThreadData *) data;
	
	// deferred mode 
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	// async mode
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

	while (1) {
		// Will block on this until
		// data can be read from the port
		//pthread_mutex_lock( &mutex1 ); 
		thData->self->waitDataIn();
		
		thData->self->notifyRead(thData->self->fd);
		//pthread_mutex_unlock( &mutex1 );				
		pthread_yield();
	}

	pthread_exit(0);
}

void M3FdNotifier::checkOutput(void* data) {
	ThreadData* thData = (ThreadData *) data;

	// deferred mode 
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	// async mode
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	
	while (1) {
		// Will block on this until
		// data can be read from the port
		//std::cout<<"WaitingDataOut"<<std::endl;
		//pthread_mutex_lock( &mutex1 ); 
		thData->self->waitDataOut();
		//std::cout<<"WaitingDataOut-DONE"<<std::endl;
		printf("start write\n");
		
		thData->self->notifyWrite(thData->self->fd);
		//pthread_mutex_unlock( &mutex1 );
		printf("end write\n");
		// Sleep 1 ms. At least
		usleep(1000);

		pthread_yield();
	}
	
	pthread_exit(0);
}

void M3FdNotifier::start() throw (std::logic_error) {
	threadData.threadNumber = 1;
	threadData.self = this;
	
	
	
	
	// TODO : Check if there is no thread already running
	pthread_create(&readThread, NULL, (void *(*)(void*)) (M3FdNotifier::checkInput),
			(void *) &threadData);

	pthread_create(&writeThread, NULL, (void *(*)(void*)) (M3FdNotifier::checkOutput),
			(void *) &threadData);
}

void M3FdNotifier::stop() throw (std::logic_error) {
    #ifdef DEBUG
    std::cerr << "Cancelling threads " << std::endl;
    #endif 
    
    pthread_cancel(readThread);
    pthread_cancel(writeThread);
    
    join();
}

void M3FdNotifier::join() throw (std::logic_error) {
  #ifdef DEBUG
  std::cerr << "Joining ... " << std::endl;
  #endif
  
  void * resRead;
  void * resWrite;
  pthread_join(readThread, &resRead);
  pthread_join(writeThread, &resWrite);

  if (resRead != PTHREAD_CANCELED || resWrite != PTHREAD_CANCELED) {
    #ifdef DEBUG
    std::cerr << "Failed to cancel threads.\n";
    #endif
  }
}

void M3FdNotifier::waitDataIn() throw (std::runtime_error) {
	fd_set input;
	FD_ZERO(&input);
	FD_SET(fd, &input);
	#ifdef DEBUG
	std::cerr << "Blocking on fd for read() ... " << fd << std::endl;
	#endif
	if (select(fd+1, &input, NULL, NULL, NULL) < 0) {
		throw std::runtime_error(std::string(
				"Failed to select() on port because : ") + std::string(
				strerror(errno)));
	}
}

void M3FdNotifier::waitDataOut() throw (std::runtime_error) {
	fd_set output;
	FD_ZERO(&output);
	FD_SET(fd, &output);

	if (select(fd+1, NULL, &output, NULL, NULL) < 0) {
		throw std::runtime_error(std::string(
				"Failed to select() on port because : ") + std::string(
				strerror(errno)));
	}
}
