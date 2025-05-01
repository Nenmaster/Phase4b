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
int diskMailbox[NUM_DISKS];
int diskInfo[NUM_DISKS][3];
int diskHeadPosition[NUM_DISKS];
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
void addDiskRequest(DiskRequest *req);
DiskRequest *getNextDiskRequest(int unit);
void diskSizeHandler(USLOSS_Sysargs *args);
void diskReadHandler(USLOSS_Sysargs *args);
void diskWriteHandler(USLOSS_Sysargs *args);
void diskHandler(int type, void *arg);
int waitDiskInterrupt(int unit);

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
    diskMailbox[i] = MboxCreate(10, sizeof(int));
    diskReadyMbox[i] = MboxCreate(1, sizeof(int));
    diskQueue[i] = NULL;
    diskHeadPosition[i] = 0;
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
  MboxCondSend(diskMailbox[unit], &status, sizeof(int)); // wake up DiskDriver
}

int DiskDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;

  // Initialize disk parameters
  diskInfo[unit][0] = 512;    // sector size
  diskInfo[unit][1] = 16;     // sectors per track
  diskHeadPosition[unit] = 0; // Start at track 0
                              //
  // Get number of tracks
  USLOSS_DeviceRequest trackReq;
  trackReq.opr = USLOSS_DISK_TRACKS;
  trackReq.reg1 = &diskInfo[unit][2];
  trackReq.reg2 = NULL;

  USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &trackReq);

  status = waitDiskInterrupt(unit);

  if (status == USLOSS_DEV_READY) {
    diskReady[unit] = true;
    int dummy = 1;
    MboxSend(diskReadyMbox[unit], &dummy, sizeof(int));
  } else {
    diskReady[unit] = false;
  }

  while (1) {
    // Wait for a request if queue is empty
    //
    if (diskQueue[unit] == NULL) {
      int dummy;
      MboxRecv(diskMailbox[unit], &dummy, sizeof(int));
      continue;
    }

    // Get the next request
    DiskRequest *req = getNextDiskRequest(unit);
    if (req == NULL) {
      printf("[DEBUG] DiskDriver(%d): Got NULL request after signal\n", unit);
      continue;
    }

    // If we need to seek, do it
    if (req->track != diskHeadPosition[unit]) {
      if (req->track < 0 || req->track >= diskInfo[unit][2]) {
        USLOSS_Console("ERROR: Invalid track number %d for unit %d\n",
                       req->track, unit);
        continue; // Skip this request
      }

      USLOSS_DeviceRequest seek;
      seek.opr = USLOSS_DISK_SEEK;
      seek.reg1 = (void *)(long)(req->track);
      seek.reg2 = NULL;

      USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &seek);
      status = waitDiskInterrupt(unit);

      // Update head position
      diskHeadPosition[unit] = req->track;
    }

    // Execute the request
    if (req->buffer == NULL) {
      req->status = -1;
      unblockProc(req->pid);

      kernSemV(diskSemaphores[req->unit]);

      free(req);
      continue;
    }

    if (req->opType == 1) { // WRITE
      void *kernelBuf = malloc(req->sectors * diskInfo[unit][0]);
      if (kernelBuf == NULL) {
        USLOSS_Console("DiskDriver(%d): malloc failed for WRITE\n", unit);
        req->status = -1;
        unblockProc(req->pid);
        continue;
      }

      memcpy(kernelBuf, req->buffer, req->sectors * diskInfo[unit][0]);

      USLOSS_DeviceRequest write;
      write.opr = USLOSS_DISK_WRITE;
      write.reg1 = (void *)(long)req->first;
      write.reg2 = kernelBuf;

      int result = USLOSS_DeviceOutput(USLOSS_DISK_DEV, req->unit, &write);
      if (result != USLOSS_DEV_OK) {
        USLOSS_Console(
            "[ERROR] Disk WRITE failed: USLOSS_DeviceOutput returned %d\n",
            result);
        USLOSS_Console("         unit=%d, track=%d, sector=%d, sectors=%d\n",
                       req->unit, req->track, req->first, req->sectors);
      }

      status = waitDiskInterrupt(unit);

      free(kernelBuf);
      req->status = status;
      kernSemV(diskSemaphores[unit]);
    }

    if (req->opType == 0) { // READ
      void *kernelBuf = malloc(req->sectors * diskInfo[unit][0]);
      if (kernelBuf == NULL) {
        USLOSS_Console("DiskDriver(%d): malloc failed for READ\n", unit);
        req->status = -1;
        unblockProc(req->pid);
        continue;
      }

      USLOSS_DeviceRequest read;
      read.opr = USLOSS_DISK_READ;
      read.reg1 = (void *)(long)req->first;
      read.reg2 = kernelBuf;

      int result = USLOSS_DeviceOutput(USLOSS_DISK_DEV, req->unit, &read);
      if (result != USLOSS_DEV_OK) {
        USLOSS_Console(
            "[ERROR] Disk WRITE failed: USLOSS_DeviceOutput returned %d\n",
            result);
        USLOSS_Console("         unit=%d, track=%d, sector=%d, sectors=%d\n",
                       req->unit, req->track, req->first, req->sectors);
      }

      status = waitDiskInterrupt(unit);

      memcpy(req->buffer, kernelBuf, req->sectors * diskInfo[unit][0]);
      free(kernelBuf);
      req->status = status;
      kernSemV(diskSemaphores[unit]);
    }

    unblockProc(req->pid);

    // After finishing this request, check if we need to reset head position
    if (diskQueue[unit] != NULL &&
        diskQueue[unit]->position >= diskInfo[unit][2]) {
      // We have requests for the next sweep, reset head position
      diskHeadPosition[unit] = 0;

      // Adjust all request positions
      DiskRequest *curr = diskQueue[unit];
      while (curr != NULL) {
        if (curr->position >= diskInfo[unit][2]) {
          curr->position -= diskInfo[unit][2];
        }
        curr = curr->next;
      }
    }
  }

  return 0;
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

  args->arg1 = (void *)(long)diskInfo[unit][0];
  args->arg2 = (void *)(long)diskInfo[unit][1];
  args->arg3 = (void *)(long)diskInfo[unit][2];
  args->arg4 = (void *)(long)0;
}

void diskReadHandler(USLOSS_Sysargs *args) {
  void *buffer = args->arg1;
  int sectors = (int)(long)args->arg2;
  int track = (int)(long)args->arg3;
  int first = (int)(long)args->arg4;
  int unit = (int)(long)args->arg5;

  if (!buffer || sectors <= 0 || unit < 0 || unit >= NUM_DISKS) {
    printf("DISKREADHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  if (track < 0 || track >= diskInfo[unit][2]) {
    printf("DISKREADHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  if (first < 0 || first + sectors > diskInfo[unit][1]) {
    printf("DISKREADHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(diskSemaphores[unit]);

  DiskRequest *req = malloc(sizeof(DiskRequest));
  req->opType = 0; // read
  req->buffer = buffer;
  req->unit = unit;
  req->track = track;
  req->first = first;
  req->sectors = sectors;
  req->pid = getpid();
  req->next = NULL;

  addDiskRequest(req);

  blockMe();

  args->arg1 = (void *)(long)req->status;
  args->arg4 = (void *)(long)0;
  free(req);
}

void diskWriteHandler(USLOSS_Sysargs *args) {
  void *buffer = args->arg1;
  int sectors = (int)(long)args->arg2;
  int track = (int)(long)args->arg3;
  int first = (int)(long)args->arg4;
  int unit = (int)(long)args->arg5;

  if (buffer == NULL) {
    printf("ERROR: buffer is NULL in diskWriteHandler\n");
    return -1;
  }

  if (!buffer || sectors <= 0 || unit < 0 || unit >= NUM_DISKS) {
    args->arg4 = (void *)(long)-1;
    printf("DISKWIRTEHANDLER failed check\n");
    return;
  }

  if (track < 0 || track >= diskInfo[unit][2]) {
    printf("DISKWIRTEHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  if (first < 0 || first + sectors > diskInfo[unit][1]) {
    printf("DISKWIRTEHANDLER failed check\n");
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(diskSemaphores[unit]);
  DiskRequest *req = malloc(sizeof(DiskRequest));
  req->opType = 1; // write
  req->buffer = buffer;
  req->unit = unit;
  req->track = track;
  req->first = first;
  req->sectors = sectors;
  req->pid = getpid();
  req->next = NULL;

  addDiskRequest(req);

  blockMe();

  args->arg1 = (void *)(long)req->status;
  args->arg4 = (void *)(long)0;
  free(req);
}

// Helpers

// Inserts a new sleep request into the queue in ascending order by wakeup time
void enqueueSleepEntry(int pid, int wakeupTime) {
  SleepEntry *newReq = malloc(sizeof(SleepEntry));
  newReq->pid = pid;
  newReq->wakeupTime = wakeupTime;
  newReq->next = NULL;

  if (sleepEntryQueue == NULL || sleepEntryQueue->wakeupTime >= wakeupTime) {
    newReq->next = sleepEntryQueue;
    sleepEntryQueue = newReq;
    return;
  }

  SleepEntry *curr = sleepEntryQueue;
  while (curr->next && curr->next->wakeupTime < wakeupTime) {
    curr = curr->next;
  }
  newReq->next = curr->next;
  curr->next = newReq;
}

// Unblocks and removes all processes from the sleep queue whose wakeup time has
// passed
void wakeUpProc() {
  while (sleepEntryQueue && sleepEntryQueue->wakeupTime <= clockTickCounter) {
    SleepEntry *tmp = sleepEntryQueue;
    sleepEntryQueue = sleepEntryQueue->next;
    unblockProc(tmp->pid);
    free(tmp);
  }
}

void addDiskRequest(DiskRequest *req) {
  int unit = req->unit;

  // Set the current head position if needed
  if (diskHeadPosition[unit] < 0) {
    diskHeadPosition[unit] = 0;
  }

  // Implement C-SCAN algorithm
  // If the request is for a track less than the current head position,
  // it will be serviced in the next sweep
  if (req->track < diskHeadPosition[unit]) {
    req->position =
        req->track +
        diskInfo[unit][2]; // Add total tracks to position for next sweep

  } else {
    req->position = req->track; // Current sweep
  }
  // Insert into ordered list
  if (diskQueue[unit] == NULL) {
    diskQueue[unit] = req;
    req->next = NULL;
  } else {
    DiskRequest *curr = diskQueue[unit];
    DiskRequest *prev = NULL;

    // Find insertion point based on position
    while (curr != NULL && curr->position <= req->position) {
      prev = curr;
      curr = curr->next;
    }

    if (prev == NULL) {
      // Insert at head
      req->next = diskQueue[unit];
      diskQueue[unit] = req;
    } else {
      // Insert after prev
      req->next = curr;
      prev->next = req;
    }
  }

  DiskRequest *temp = diskQueue[unit];

  // Signal the disk driver
  int dummy = 0;
  MboxSend(diskMailbox[unit], &dummy, sizeof(int));
}

DiskRequest *getNextDiskRequest(int unit) {
  DiskRequest *req = diskQueue[unit];
  if (req)
    diskQueue[unit] = req->next;
  return req;
}

int waitDiskInterrupt(int unit) {
  int status;

  int result = MboxRecv(diskMailbox[unit], &status, sizeof(int));
  return (result == sizeof(int)) ? status : -1;
}
