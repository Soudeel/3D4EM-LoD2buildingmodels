
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/



// Magic Software, Inc.
// http://www.magic-software.com
// Copyright (c) 2000-2002.  All Rights Reserved
//
// Source code from Magic Software is supplied under the terms of a license
// agreement and may not be copied or disclosed except in accordance with the
// terms of that agreement.  The various license agreements may be found at
// the Magic Software web site.  This file is subject to the license
//
// FREE SOURCE CODE
// http://www.magic-software.com/License/free.pdf

#include "MgcCommand.h"
#include <string.h>
#include <stdio.h>
#include <iostream>
using namespace std;

char Command::ms_acOptionNotFound[] = "option not found";
char Command::ms_acArgumentRequired[] = "option requires an argument";
char Command::ms_acArgumentOutOfRange[] = "argument out of range";
char Command::ms_acFilenameNotFound[] = "filename not found";

//----------------------------------------------------------------------------
Command::Command (int iQuantity, char** apcArgument)
{
    m_iQuantity = iQuantity;
    m_apcArgument = apcArgument;
    m_acCmdline = NULL;
    m_abUsed = NULL;

    Initialize();
}
//----------------------------------------------------------------------------
Command::Command (char* acCmdline)
{
    class Argument
    {
    public:
        char* m_pcItem;
        Argument* m_pkNext;
    };

    m_iQuantity = 0;
    m_apcArgument = NULL;

    if ( acCmdline == NULL || strlen(acCmdline) == 0 )
        return;

    m_acCmdline = new char[strlen(acCmdline)+1];
    strcpy(m_acCmdline,acCmdline);

    char* pcToken = strtok(m_acCmdline," \t");
    Argument* pkList = NULL;

    while ( pcToken )
    {
        m_iQuantity++;

        Argument* pkCurrent = new Argument;
        pkCurrent->m_pcItem = pcToken;
        pkCurrent->m_pkNext = pkList;
        pkList = pkCurrent;

        pcToken = strtok(0," \t");
    }

    m_iQuantity++;
    m_apcArgument = new char*[m_iQuantity];
    m_apcArgument[0] = m_acCmdline;
    int i = m_iQuantity-1;
    while ( pkList )
    {
        m_apcArgument[i--] = pkList->m_pcItem;
        
        Argument* pkSave = pkList->m_pkNext;
        delete pkList;
        pkList = pkSave;
    }

    Initialize();
}
//----------------------------------------------------------------------------
Command::~Command ()
{
    delete[] m_abUsed;

    if ( m_acCmdline )
    {
        delete[] m_apcArgument;
        delete[] m_acCmdline;
    }
}
//----------------------------------------------------------------------------
void Command::Initialize ()
{
    m_abUsed = new bool[m_iQuantity];
    memset(m_abUsed,false,m_iQuantity*sizeof(bool));

    m_dSmall = 0.0;
    m_dLarge = 0.0;
    m_bMinSet = false;
    m_bMaxSet = false;
    m_bInfSet = false;
    m_bSupSet = false;

    m_acLastError = NULL;
}
//----------------------------------------------------------------------------
int Command::ExcessArguments ()
{
    // checks to see if any command line arguments were not processed
    for (int i = 1; i < m_iQuantity; i++)
    {
        if ( !m_abUsed[i] )
            return i;
    }

    return 0;
}
//----------------------------------------------------------------------------
Command& Command::Min (double dValue)
{
    m_dSmall = dValue;
    m_bMinSet = true;
    return *this;
}
//----------------------------------------------------------------------------
Command& Command::Max (double dValue)
{
    m_dLarge = dValue;
    m_bMaxSet = true;
    return *this;
}
//----------------------------------------------------------------------------
Command& Command::Inf (double dValue)
{
    m_dSmall = dValue;
    m_bInfSet = true;
    return *this;
}
//----------------------------------------------------------------------------
Command& Command::Sup (double dValue)
{
    m_dLarge = dValue;
    m_bSupSet = true;
    return *this;
}
//----------------------------------------------------------------------------
int Command::Boolean (char* acName)
{
    bool bValue = false;
    return Boolean(acName,bValue);
}
//----------------------------------------------------------------------------
int Command::Boolean (char* acName, bool& rbValue)
{
    m_acLastError = NULL;
	int iMatchFound = 0;
    rbValue = false;
    for (int i = 1; i < m_iQuantity; i++)
    {
        char* pcTmp = m_apcArgument[i];
        if ( !m_abUsed[i] && pcTmp[0] == '-' && strcmp(acName,++pcTmp) == 0 )
        {
            m_abUsed[i] = true;
            iMatchFound = i;
            rbValue = true;
            break;
        }
    }

    if ( iMatchFound == 0 )
        m_acLastError = ms_acOptionNotFound;

    return iMatchFound;
}
//----------------------------------------------------------------------------
int Command::Integer (char* acName, int& riValue)
{
    m_acLastError = NULL;
	int iMatchFound = 0;
    for (int i = 1; i < m_iQuantity; i++)
    {
        char* pcTmp = m_apcArgument[i];
        if ( !m_abUsed[i] && pcTmp[0] == '-' && strcmp(acName,++pcTmp) == 0 )
        {
            // get argument
            pcTmp = m_apcArgument[i+1];
            if ( m_abUsed[i+1] || (pcTmp[0] == '-' && !isdigit(pcTmp[1])) )
            {
                m_acLastError = ms_acArgumentRequired;
                return 0;
            }
            sscanf(pcTmp,"%d",&riValue);
            if ( (m_bMinSet && riValue < m_dSmall)
            ||   (m_bMaxSet && riValue > m_dLarge)
            ||   (m_bInfSet && riValue <= m_dSmall)
            ||   (m_bSupSet && riValue >= m_dLarge) )
            {
                m_acLastError = ms_acArgumentOutOfRange;
                return 0;
            }
            m_abUsed[i] = true;
            m_abUsed[i+1] = true;
            iMatchFound = i;
            break;
        }
    }

    m_bMinSet = false;
    m_bMaxSet = false;
    m_bInfSet = false;
    m_bSupSet = false;

    if ( iMatchFound == 0 )
        m_acLastError = ms_acOptionNotFound;

    return iMatchFound;
}
//----------------------------------------------------------------------------
int Command::Float (char* acName, float& rfValue)
{
    m_acLastError = NULL;
	int iMatchFound = 0;
    for (int i = 1; i < m_iQuantity; i++)
    {
        char* pcTmp = m_apcArgument[i];
        if ( !m_abUsed[i] && pcTmp[0] == '-' && strcmp(acName,++pcTmp) == 0 )
        {
            // get argument
            pcTmp = m_apcArgument[i+1];
            if ( m_abUsed[i+1] || (pcTmp[0] == '-' && !isdigit(pcTmp[1])) )
            {
                m_acLastError = ms_acArgumentRequired;
                return 0;
            }
            sscanf(pcTmp,"%f",&rfValue);
            if ( (m_bMinSet && rfValue < m_dSmall)
            ||   (m_bMaxSet && rfValue > m_dLarge)
            ||   (m_bInfSet && rfValue <= m_dSmall)
            ||   (m_bSupSet && rfValue >= m_dLarge) )
            {
                m_acLastError = ms_acArgumentOutOfRange;
                return 0;
            }
            m_abUsed[i] = true;
            m_abUsed[i+1] = true;
            iMatchFound = i;
            break;
        }
    }

    m_bMinSet = false;
    m_bMaxSet = false;
    m_bInfSet = false;
    m_bSupSet = false;

    if ( iMatchFound == 0 )
        m_acLastError = ms_acOptionNotFound;

    return iMatchFound;
}
//----------------------------------------------------------------------------
int Command::Double (char* acName, double& rdValue)
{
    m_acLastError = NULL;
	int iMatchFound = 0;
    for (int i = 1; i < m_iQuantity; i++)
    {
        char* pcTmp = m_apcArgument[i];
        if ( !m_abUsed[i] && pcTmp[0] == '-' && strcmp(acName,++pcTmp) == 0 )
        {
            // get argument
            pcTmp = m_apcArgument[i+1];
            if ( m_abUsed[i+1] || (pcTmp[0] == '-' && !isdigit(pcTmp[1])) )
            {
                m_acLastError = ms_acArgumentRequired;
                return 0;
            }
            sscanf(pcTmp,"%lf",&rdValue);
            if ( (m_bMinSet && rdValue < m_dSmall)
            ||   (m_bMaxSet && rdValue > m_dLarge)
            ||   (m_bInfSet && rdValue <= m_dSmall)
            ||   (m_bSupSet && rdValue >= m_dLarge) )
            {
                m_acLastError = ms_acArgumentOutOfRange;
                return 0;
            }
            m_abUsed[i] = true;
            m_abUsed[i+1] = true;
            iMatchFound = i;
            break;
        }
    }

    m_bMinSet = false;
    m_bMaxSet = false;
    m_bInfSet = false;
    m_bSupSet = false;

    if ( iMatchFound == 0 )
        m_acLastError = ms_acOptionNotFound;

    return iMatchFound;
}
//----------------------------------------------------------------------------
int Command::String (char* acName, char*& racValue)
{
    m_acLastError = NULL;
	int iMatchFound = 0;
    for (int i = 1; i < m_iQuantity; i++)
    {
        char* pcTmp = m_apcArgument[i];
        if ( !m_abUsed[i] && pcTmp[0] == '-' && strcmp(acName,++pcTmp) == 0 )
        {
            // get argument
            pcTmp = m_apcArgument[i+1];
            if ( m_abUsed[i+1] || pcTmp[0] == '-' )
            {
                m_acLastError = ms_acArgumentRequired;
                return 0;
            }

            racValue = new char[strlen(pcTmp)+1];
            strcpy(racValue,pcTmp);
            m_abUsed[i] = true;
            m_abUsed[i+1] = true;
            iMatchFound = i;
            break;
        }
    }

    if ( iMatchFound == 0 )
        m_acLastError = ms_acOptionNotFound;

    return iMatchFound;
}
//----------------------------------------------------------------------------
int Command::Filename (char*& racName)
{
    m_acLastError = NULL;
	int iMatchFound = 0;
    for (int i = 1; i < m_iQuantity; i++)
    {
        char* pcTmp = m_apcArgument[i];
        if ( !m_abUsed[i] && pcTmp[0] != '-' )
        {
            racName = new char[strlen(pcTmp)+1];
            strcpy(racName,pcTmp);
            m_abUsed[i] = true;
            iMatchFound = i;
            break;
        }
    }

    if ( iMatchFound == 0 )
        m_acLastError = ms_acFilenameNotFound;

    return iMatchFound;
}
//----------------------------------------------------------------------------
const char* Command::GetLastError ()
{
    return m_acLastError;
}
//----------------------------------------------------------------------------
int Command::String(char* acName,string & racValue)
{
	m_acLastError = NULL;
	char* buff = NULL;
	int status = String(acName,buff);

	if(buff)
	{
		racValue = buff;
		delete[] buff;
		m_acLastError = NULL;
	}

	return status;
}
			
	
	
int Command::Print()const
{
	for (int i = 1; i < m_iQuantity; i++)
	{
		cerr<<m_apcArgument[i]<< "**  ";
	}
	cerr<<endl;
}
	
int Command::Filename (std::string& fileName)
{
	char* buff = NULL;
	int status = Filename(buff);

	if(buff)
	{
		fileName = buff;
		delete[] buff;
		m_acLastError = NULL;

	}
	return status;
}
