/*
 *
 *Programmers: Omar Mendivil & Ayman Mohamed
 *Phase4a
 * Implements kernel side handlers for Sleep(), TermRead(), and TermWrite()
 * using clock and terminal device drivers.
 *
 * */

#include <assert.h>
#include <phase1.h>
#include <phase2.h>
#include <phase3.h>
#include <phase3_kernelInterfaces.h>
#include <phase4.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usloss.h>
#include <usyscall.h>

#define NUM_DISKS 2
#define WRITE 1
#define READ 0

typedef struct SleepEntry {
  int wakeupTime;
  int pid;
  struct SleepEntry *next;
} SleepEntry;

///// NEW STRUCT ADDED FOR 4B////////
typedef struct DiskRequest {
  int opType; // 0=read, 1=write
  void *buffer;
  int unit;
  int track;
  int first;
  int sectors;
  int status;
  int pid;
  struct DiskRequest *next;
  int position;
} DiskRequest;

// Globals
SleepEntry *sleepEntryQueue = NULL;
int clockMailbox;
int clockTickCounter = 0;

int termWriteLock[4];
int termReadMailbox[4];
int termInterruptMailbox[4];
int termWritePid[USLOSS_TERM_UNITS];

char writeBuff[4][MAXLINE];
int writeLen[4];
int writIdx[4];
int writeWaiting[4];

char lineBuff[4][MAXLINE];
int lineLen[4];

/// NEW STUFF ADDED FOR 4B ///////
DiskRequest *diskQueue[NUM_DISKS];
int diskRequestMbox[NUM_DISKS];
int diskInterruptMbox[NUM_DISKS];
int diskData[NUM_DISKS][3];
int currTrackPos[NUM_DISKS];
int diskSemaphores[NUM_DISKS];
bool diskReady[NUM_DISKS] = {false};
int diskReadyMbox[NUM_DISKS];

// ProtoTypes
void enqueueSleepEntry(int pid, int wakeupTime);
void wakeUpProc();
int ClockDriver(void *arg);
int TerminalDriver(void *arg);
void sleepHandler(USLOSS_Sysargs *args);
void termWriteHandler(USLOSS_Sysargs *args);
void termReadHandler(USLOSS_Sysargs *args);
void termHandler(int type, void *arg);

//////// NWEW STUFF ADDED FOR 4B ////////
int DiskDriver(void *arg);
void diskSizeHandler(USLOSS_Sysargs *args);
void diskReadHandler(USLOSS_Sysargs *args);
void diskWriteHandler(USLOSS_Sysargs *args);
void diskHandler(int type, void *arg);
int waitDiskInterrupt(int unit);
void enqueueDiskRequest(DiskRequest *request);
DiskRequest *dequeueDiskRequest(int unit);

void phase4_start_service_processes(void) {
  spork("ClockDriver", ClockDriver, NULL, USLOSS_MIN_STACK, 1);
  for (int i = 0; i < 4; i++) {
    spork("TermDriver", TerminalDriver, (void *)(long)i, USLOSS_MIN_STACK, 1);
  }

  for (int i = 0; i < 4; i++) {
    int control = USLOSS_TERM_CTRL_RECV_INT(0) | USLOSS_TERM_CTRL_XMIT_INT(0);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, i, (void *)(long)control);
  }

  for (int i = 0; i < NUM_DISKS; i++) {
    spork("DiskDriver", DiskDriver, (void *)(long)i, USLOSS_MIN_STACK, 2);
  }
}

void phase4_init(void) {
  systemCallVec[SYS_SLEEP] = sleepHandler;
  systemCallVec[SYS_TERMWRITE] = termWriteHandler;
  systemCallVec[SYS_TERMREAD] = termReadHandler;
  USLOSS_IntVec[USLOSS_TERM_INT] = termHandler;

  clockMailbox = MboxCreate(1, 0);
  for (int i = 0; i < 4; i++) {
    int semId;
    termWriteLock[i] = kernSemCreate(1, &semId);
    termReadMailbox[i] = MboxCreate(10, MAXLINE);
    termInterruptMailbox[i] = MboxCreate(8, sizeof(int));
    termWritePid[i] = -1;
    writeLen[i] = 0;
    writIdx[i] = 0;
    writeWaiting[i] = 0;
    lineLen[i] = 0;
  }

  systemCallVec[SYS_DISKSIZE] = diskSizeHandler;
  systemCallVec[SYS_DISKREAD] = diskReadHandler;
  systemCallVec[SYS_DISKWRITE] = diskWriteHandler;
  USLOSS_IntVec[USLOSS_DISK_INT] = diskHandler;

  for (int i = 0; i < NUM_DISKS; i++) {
    diskRequestMbox[i] = MboxCreate(10, sizeof(int));
    diskInterruptMbox[i] = MboxCreate(10, sizeof(int));
    diskReadyMbox[i] = MboxCreate(1, sizeof(int));
    diskQueue[i] = NULL;
    currTrackPos[i] = 0;
    int semId;
    kernSemCreate(1, &semId); // Initial value 1 = unlocked
    diskSemaphores[i] = semId;
  }
}

// Waits for clock interrupts and wakes up any processes whose
// sleep time has finished
int ClockDriver(void *arg) {
  int status;
  while (1) {
    waitDevice(USLOSS_CLOCK_DEV, 0, &status);
    clockTickCounter++;
    wakeUpProc();
  }
  return 0;
}

// sends device status to terminal driver via mailbox
void termHandler(int type, void *arg) {
  int unit = (int)(long)arg;
  int status;
  USLOSS_DeviceInput(USLOSS_TERM_DEV, unit, &status);
  MboxCondSend(termInterruptMailbox[unit], &status, sizeof(int));
}

// Handles RECV and XMIT interrupts for one terminal and
// buffers incoming characters and manages transmition
int TerminalDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;
  while (1) {
    if (MboxRecv(termInterruptMailbox[unit], &status, sizeof(int)) !=
        sizeof(int))
      continue;

    if (USLOSS_TERM_STAT_XMIT(status) == USLOSS_DEV_READY) {
      kernSemP(termWriteLock[unit]);
      if (writeWaiting[unit]) {
        if (writIdx[unit] < writeLen[unit]) {
          char ch = writeBuff[unit][writIdx[unit]++];
          kernSemV(termWriteLock[unit]);
          int control = USLOSS_TERM_CTRL_CHAR(0, ch);
          control = USLOSS_TERM_CTRL_XMIT_INT(control);
          control = USLOSS_TERM_CTRL_RECV_INT(control);
          control |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
          USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)control);
        } else {
          int pidToUnblock = termWritePid[unit];
          writeWaiting[unit] = 0;
          termWritePid[unit] = -1;
          kernSemV(termWriteLock[unit]);
          if (pidToUnblock != -1)
            unblockProc(pidToUnblock);
          int control =
              USLOSS_TERM_CTRL_RECV_INT(0) | USLOSS_TERM_CTRL_XMIT_INT(0);
          USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)control);
        }
      } else
        kernSemV(termWriteLock[unit]);
    }

    if (USLOSS_TERM_STAT_RECV(status) == USLOSS_DEV_BUSY) {
      char ch = USLOSS_TERM_STAT_CHAR(status);
      if (lineLen[unit] < MAXLINE)
        lineBuff[unit][lineLen[unit]++] = ch;
      if (ch == '\n' || lineLen[unit] == MAXLINE) {
        lineBuff[unit][lineLen[unit]] = '\0';
        MboxSend(termReadMailbox[unit], lineBuff[unit], lineLen[unit]);
        lineLen[unit] = 0;
      }
      int control = USLOSS_TERM_CTRL_RECV_INT(0);
      kernSemP(termWriteLock[unit]);
      if (writeWaiting[unit])
        control |= USLOSS_TERM_CTRL_XMIT_INT(0);
      kernSemV(termWriteLock[unit]);
      USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)control);
    }
  }
  return 0;
}

// Sends a string to the terminal, one character at a time.
void termWriteHandler(USLOSS_Sysargs *args) {
  char *buf = (char *)args->arg1;
  int len = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;
  int pid = getpid();

  if (!buf || len < 0 || len > MAXLINE || unit < 0 || unit >= 4) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(termWriteLock[unit]);
  memcpy(writeBuff[unit], buf, len);
  writeLen[unit] = len;
  writIdx[unit] = 0;
  writeWaiting[unit] = 1;
  termWritePid[unit] = pid;
  kernSemV(termWriteLock[unit]);

  if (len > 0) {
    char ch = writeBuff[unit][writIdx[unit]++];
    int control = USLOSS_TERM_CTRL_CHAR(0, ch);
    control = USLOSS_TERM_CTRL_XMIT_INT(control);
    control = USLOSS_TERM_CTRL_RECV_INT(control);
    control |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)control);
    blockMe();
  } else {
    kernSemP(termWriteLock[unit]);
    writeWaiting[unit] = 0;
    termWritePid[unit] = -1;
    kernSemV(termWriteLock[unit]);
  }

  args->arg2 = (void *)(long)len;
  args->arg4 = (void *)(long)0;
}

// Delivers one full line from terminal input buffer to caller.
void termReadHandler(USLOSS_Sysargs *args) {
  char *buffer = (char *)args->arg1;
  int bufSize = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;
  if (!buffer || bufSize <= 0 || unit < 0 || unit > 3) {
    args->arg4 = (void *)(long)-1;
    return;
  }
  char tempBuf[MAXLINE];
  int len = MboxRecv(termReadMailbox[unit], tempBuf, MAXLINE);
  if (len > bufSize)
    len = bufSize;
  memcpy(buffer, tempBuf, len);
  args->arg2 = (void *)(long)len;
  args->arg4 = (void *)(long)0;
}

// Blocks the calling process for the specified number of seconds
// and inserts the process into the sleep queue
void sleepHandler(USLOSS_Sysargs *args) {
  int seconds = (int)(long)args->arg1;
  if (seconds < 0) {
    args->arg4 = (void *)(long)-1;
    return;
  }
  int pid = getpid();
  int wakeupTick = clockTickCounter + (seconds * 10);
  enqueueSleepEntry(pid, wakeupTick);
  blockMe();
  args->arg4 = (void *)(long)0;
}

void diskHandler(int type, void *arg) {
  int unit = (int)(long)arg;
  int status;
  USLOSS_DeviceInput(USLOSS_DISK_DEV, unit, &status);
  MboxCondSend(diskInterruptMbox[unit], &status, sizeof(int));
}

int DiskDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;

  diskData[unit][0] = 512;
  diskData[unit][1] = 16;
  currTrackPos[unit] = 0;

  USLOSS_DeviceRequest getTrack;
  getTrack.opr = USLOSS_DISK_TRACKS;
  getTrack.reg1 = &diskData[unit][2];
  getTrack.reg2 = NULL;

  USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &getTrack);
  status = waitDiskInterrupt(unit);

  if (status == USLOSS_DEV_READY) {
    diskReady[unit] = true;
    int dummy = 1;
    MboxSend(diskReadyMbox[unit], &dummy, sizeof(int));
  } else {
    diskReady[unit] = false;
  }

  while (1) {
    if (diskQueue[unit] == NULL) {
      int dummy;
      MboxRecv(diskRequestMbox[unit], &dummy, sizeof(int));
      continue;
    }

    DiskRequest *request = dequeueDiskRequest(unit);

    if (request->opType == WRITE) {
      int remaining = request->sectors;
      int currTrack = request->track;
      int currSector = request->first;
      char *src = request->buffer;

      while (remaining > 0) {
        int space = diskData[unit][1] - currSector;
        int toWrite = (remaining < space) ? remaining : space;

        if (currTrackPos[unit] != currTrack) {
          USLOSS_DeviceRequest seek;
          seek.opr = USLOSS_DISK_SEEK;
          seek.reg1 = (void *)(long)currTrack;
          seek.reg2 = NULL;
          USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &seek);
          int seekStatus = waitDiskInterrupt(unit);
          if (seekStatus != USLOSS_DEV_READY) {
            request->status = seekStatus;
            unblockProc(request->pid);
            kernSemV(diskSemaphores[unit]);
            free(request);
            continue;
          }
          currTrackPos[unit] = currTrack;
        }

        void *kernelBuf = malloc(toWrite * diskData[unit][0]);
        if (!kernelBuf) {
          USLOSS_Console("DiskDriver(%d): malloc failed for WRITE\n", unit);
          request->status = -1;
          unblockProc(request->pid);
          kernSemV(diskSemaphores[request->unit]);
          free(request);
          continue;
        }

        memcpy(kernelBuf, src, toWrite * diskData[unit][0]);

        USLOSS_DeviceRequest write;
        write.opr = USLOSS_DISK_WRITE;
        write.reg1 = (void *)(long)currSector;
        write.reg2 = kernelBuf;

        USLOSS_DeviceOutput(USLOSS_DISK_DEV, request->unit, &write);

        status = waitDiskInterrupt(unit);
        free(kernelBuf);

        remaining -= toWrite;
        src += toWrite * diskData[unit][0];
        currSector = 0;
        currTrack++;
      }

      request->status = status;
      kernSemV(diskSemaphores[unit]);
      unblockProc(request->pid);
      free(request);
      continue;
    }

    if (request->opType == READ) {
      int remaining = request->sectors;
      int currTrack = request->track;
      int currSector = request->first;
      char *dest = request->buffer;

      while (remaining > 0) {
        int space = diskData[unit][1] - currSector;
        int toRead = (remaining < space) ? remaining : space;

        if (currTrackPos[unit] != currTrack) {
          USLOSS_DeviceRequest seek;
          seek.opr = USLOSS_DISK_SEEK;
          seek.reg1 = (void *)(long)currTrack;
          seek.reg2 = NULL;
          USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &seek);
          int seekStatus = waitDiskInterrupt(unit);
          if (seekStatus != USLOSS_DEV_READY) {
            request->status = seekStatus;
            unblockProc(request->pid);
            kernSemV(diskSemaphores[unit]);
            free(request);
            continue;
          }
          currTrackPos[unit] = currTrack;
        }

        void *kernelBuf = malloc(toRead * diskData[unit][0]);
        if (!kernelBuf) {
          USLOSS_Console("DiskDriver(%d): malloc failed for READ\n", unit);
          request->status = -1;
          unblockProc(request->pid);
          kernSemV(diskSemaphores[request->unit]);
          free(request);
          continue;
        }

        USLOSS_DeviceRequest read;
        read.opr = USLOSS_DISK_READ;
        read.reg1 = (void *)(long)currSector;
        read.reg2 = kernelBuf;

        USLOSS_DeviceOutput(USLOSS_DISK_DEV, request->unit, &read);

        status = waitDiskInterrupt(unit);
        memcpy(dest, kernelBuf, toRead * diskData[unit][0]);
        free(kernelBuf);

        remaining -= toRead;
        dest += toRead * diskData[unit][0];
        currSector = 0;
        currTrack++;
      }

      request->status = status;
      kernSemV(diskSemaphores[unit]);
      unblockProc(request->pid);
      free(request);
      continue;
    }

    // Catch-all fallback
    USLOSS_Console("[ERROR] Unknown opType = %d in DiskDriver\n",
                   request->opType);
    request->status = -1;
    kernSemV(diskSemaphores[unit]);
    unblockProc(request->pid);
    free(request);
  }

  return 0; // never reached
}

void diskSizeHandler(USLOSS_Sysargs *args) {
  int unit = (int)(long)args->arg1;
  if (unit < 0 || unit >= NUM_DISKS) {
    printf("DISK SIZE HANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  if (!diskReady[unit]) {
    int dummy;
    MboxRecv(diskReadyMbox[unit], &dummy, sizeof(int));
  }

  args->arg1 = (void *)(long)diskData[unit][0];
  args->arg2 = (void *)(long)diskData[unit][1];
  args->arg3 = (void *)(long)diskData[unit][2];
  args->arg4 = (void *)(long)0;
}

void diskReadHandler(USLOSS_Sysargs *args) {
  void *buffer = args->arg1;
  int sectors = (int)(long)args->arg2;
  int track = (int)(long)args->arg3;
  int first = (int)(long)args->arg4;
  int unit = (int)(long)args->arg5;

  if (!diskReady[unit]) {
    int dummy;
    MboxRecv(diskReadyMbox[unit], &dummy, sizeof(int));
  }

  if (!buffer || sectors <= 0 || unit < 0 || unit >= NUM_DISKS) {
    printf("DISKREADHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  if (track < 0 || track >= diskData[unit][2]) {
    printf("DISKREADHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  if (first < 0 || first >= diskData[unit][1]) {
    printf("DISKREADHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(diskSemaphores[unit]);

  DiskRequest *request = malloc(sizeof(DiskRequest));
  request->opType = 0; // read
  request->buffer = buffer;
  request->unit = unit;
  request->track = track;
  request->first = first;
  request->sectors = sectors;
  request->pid = getpid();
  request->next = NULL;

  enqueueDiskRequest(request);

  blockMe();

  args->arg1 = (void *)(long)request->status;
  args->arg4 = (void *)(long)0;
}

void diskWriteHandler(USLOSS_Sysargs *args) {
  void *buffer = args->arg1;
  int sectors = (int)(long)args->arg2;
  int track = (int)(long)args->arg3;
  int first = (int)(long)args->arg4;
  int unit = (int)(long)args->arg5;

  if (!diskReady[unit]) {
    int dummy;
    MboxRecv(diskReadyMbox[unit], &dummy, sizeof(int));
  }

  if (buffer == NULL) {
    // printf("ERROR: buffer is NULL in diskWriteHandler\n");
    return;
  }

  if (!buffer || sectors <= 0 || unit < 0 || unit >= NUM_DISKS) {
    args->arg4 = (void *)(long)-1;
    // printf("DISKWIRTEHANDLER failed check\n");
    return;
  }

  // if (track < 0 || track >= diskData[unit][2]) {
  //   printf("DISKWIRTEHANDLER failed check\n");
  //   args->arg4 = (void *)(long)-1;
  //   return;
  // }

  if (first < 0 || first >= diskData[unit][1]) {
    // printf("DISKWIRTEHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(diskSemaphores[unit]);
  DiskRequest *request = malloc(sizeof(DiskRequest));
  request->opType = 1; // write
  request->buffer = buffer;
  request->unit = unit;
  request->track = track;
  request->first = first;
  request->sectors = sectors;
  request->pid = getpid();
  request->next = NULL;

  enqueueDiskRequest(request);

  blockMe();

  args->arg1 = (void *)(long)request->status;
  args->arg4 = (void *)(long)0;
}

// Helpers

// Inserts a new sleep request into the queue in ascending order by wakeup
// time
void enqueueSleepEntry(int pid, int wakeupTime) {
  SleepEntry *newRequest = malloc(sizeof(SleepEntry));
  newRequest->pid = pid;
  newRequest->wakeupTime = wakeupTime;
  newRequest->next = NULL;

  if (sleepEntryQueue == NULL || sleepEntryQueue->wakeupTime >= wakeupTime) {
    newRequest->next = sleepEntryQueue;
    sleepEntryQueue = newRequest;
    return;
  }

  SleepEntry *curr = sleepEntryQueue;
  while (curr->next && curr->next->wakeupTime < wakeupTime) {
    curr = curr->next;
  }
  newRequest->next = curr->next;
  curr->next = newRequest;
}

// Unblocks and removes all processes from the sleep queue whose wakeup time
// has passed
void wakeUpProc() {
  while (sleepEntryQueue && sleepEntryQueue->wakeupTime <= clockTickCounter) {
    SleepEntry *tmp = sleepEntryQueue;
    sleepEntryQueue = sleepEntryQueue->next;
    unblockProc(tmp->pid);
    free(tmp);
  }
}

void enqueueDiskRequest(DiskRequest *request) {
  int unit = request->unit;

  // Set the current head position if needed
  if (currTrackPos[unit] < 0) {
    currTrackPos[unit] = 0;
  }

  // Implement C-SCAN algorithm
  // If the request is for a track less than the current head position,
  // it will be serviced in the next sweep
  if (request->track < currTrackPos[unit]) {
    request->position = request->track + diskData[unit][2];
  } else {
    request->position = request->track; // Current sweep
  }

  if (diskQueue[unit] == NULL) {
    diskQueue[unit] = request;
    request->next = NULL;
  } else {
    DiskRequest *curr = diskQueue[unit];
    DiskRequest *prev = NULL;

    // Find insertion point based on position
    while (curr != NULL && curr->position <= request->position) {
      prev = curr;
      curr = curr->next;
    }

    if (prev == NULL) {
      // Insert at head
      request->next = diskQueue[unit];
      diskQueue[unit] = request;
    } else {
      // Insert after prev
      request->next = curr;
      prev->next = request;
    }
  }

  // Signal the disk driver
  int dummy = 0;
  MboxSend(diskRequestMbox[unit], &dummy, sizeof(int));
}

DiskRequest *dequeueDiskRequest(int unit) {
  DiskRequest *request = diskQueue[unit];
  if (request)
    diskQueue[unit] = request->next;
  return request;
}

int waitDiskInterrupt(int unit) {
  int status;

  int result = MboxRecv(diskInterruptMbox[unit], &status, sizeof(int));
  return (result == sizeof(int)) ? status : -1;
}
