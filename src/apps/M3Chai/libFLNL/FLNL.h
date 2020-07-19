/*! \mainpage FLNL client/server library documentation
 *
 *  \section intro_sec Inroduction
 *  This documentation describes the different classes of a network client/server library. This library ables to send double values between two systems (a client and a server). <BR>
 * A server have to be created in a first time, waiting for a connection. Then a client on a remote machine could connect to it (with ip address) and so the communication could began.<BR>
 * The library could be compiled under Linux and Windows (choose the corresponding target in Code::Blocks project).
 *
 *  \subsection sub_linux Linux
 *  For Linux the project generates the file libFLNL.so in the unix directory.
 *  \subsection sub_win Windows
 *  For Windows the project generates the files libFLNL.a and libFLNL.dll in the win directory.
 * <BR>
 *
 *  \section contact Contact
 *  vincent.crocher@isir.upmc.fr
 *  <BR><BR>
 *
 */

/**
 * \file FLNL.h
 * \brief Network classes declaration
 * \author Vincent Crocher
 * \version 0.8
 * \date July 2020
 *
 *
 */

#ifndef FLNL_H_INCLUDED
#define FLNL_H_INCLUDED


#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>


#ifdef WIN32
    #define WINDOWS
#endif
#ifdef X64
    #define WINDOWS
#endif

#ifdef WINDOWS
    #include <winsock.h>
    #include "pthread.h"
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <pthread.h>
#endif


void * receiving(void *c);

//! A network base class able to send and receive data (fixed number of doubles)
class baseSocket
{
    friend void * receiving(void *c);

    public:
        //!@name Constructor and destructor
        //@{
            baseSocket(unsigned char nb_values_to_send, unsigned char nb_values_to_receive);
            ~baseSocket();
        //@}

        //!@name Connecting and disconnecting methods
        //@{
            virtual int Connect(char * addr) = 0;
            virtual int Reconnect() {return 0;};
            virtual int Disconnect();
            bool IsConnected();
        //@}

        //!@name Sending methods
        //@{
            int Send(double * values);
        //@}

        //!@name Receiving methods
        //@{
            bool IsReceivedValues();
            double * GetReceivedValues();
        //@}

    protected:
        int Socket;                         //!< Local socket
        struct sockaddr_in sin;             //!< Socket parameters structure
        bool Connected;                     //!< TRUE if client is connected to a server, FALSE otherwise
        unsigned char NbValuesToSend;       //!< Number of double values the client send to the server
        unsigned char NbValuesToReceive;    //!< Number of double values the client receive from the server
        const unsigned char InitCode = 'V';
        double * ReceivedValues;            //!< Tab of the last received value from the server
        bool IsValues;                      //!< TRUE if last values are received (since last GetReceivedValues()), FALSE otherwise
        pthread_t ReceivingThread;          //!< Receiving pthread
};


//! A network client able to send and receive data (doubles)
class client: public baseSocket
{
    public:
        //!@name Constructor and destructor
        //@{
            client(unsigned char nb_values_to_send, unsigned char nb_values_to_receive): baseSocket(nb_values_to_send, nb_values_to_receive){};
        //@}

        //!@name Connecting and disconnecting methods
        //@{
            int Connect(char * addr);
        //@}
};

void * accepting(void *c);

//! A network server able to send and receive data (doubles)
class server: public baseSocket
{
    friend void * accepting(void *c);

    public:
        //!@name Constructor and destructor
        //@{
            server(unsigned char nb_values_to_send, unsigned char nb_values_to_receive): baseSocket(nb_values_to_send, nb_values_to_receive) {
                //Initialise and allocates privates
                Waiting=false;
                ReceivedValues=new double[NbValuesToReceive];
            }
        //@}

        //!@name Connecting and disconnecting methods
        //@{
            int Connect(char * addr);
            int Disconnect();
            int Reconnect();
        //@}

    private:
        int ServerSocket;                         //!< Server (accepting) socket
        bool Waiting;                       //!< TRUE if server is currently waiting for incoming client, FALSE otherwise
        pthread_t AcceptingThread;          //!< Accepting connection (from client) pthread
};

#endif // FLNL_H_INCLUDED
