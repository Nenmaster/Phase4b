/*
 *
 *Programmers: Omar Mendivil & Ayman Mohamed
 *Phase4a
 * Implements kernel side handlers for Sleep(), TermRead(), and TermWrite()
 * using clock and terminal device drivers.
 *
 * */

#include <phase1.h>
#include <phase2.h>
#include <phase3.h>
#include <phase3_kernelInterfaces.h>
#include <phase4.h>
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

DiskRequest *diskQueue[NUM_DISKS];
int diskMailbox[NUM_DISKS];
int diskInfo[NUM_DISKS][3];

// ProtoTypes
void enqueueSleepEntry(int pid, int wakeupTime);
void wakeUpProc();
int ClockDriver(void *arg);
int TerminalDriver(void *arg);
void sleepHandler(USLOSS_Sysargs *args);
void termWriteHandler(USLOSS_Sysargs *args);
void termReadHandler(USLOSS_Sysargs *args);
void termHandler(int type, void *arg);
int DiskDriver(void *arg);
void addDiskRequest(DiskRequest *req);
DiskRequest *getNextDiskRequest(int unit);
void diskSizeHandler(USLOSS_Sysargs *args);
void diskReadHandler(USLOSS_Sysargs *args);
void diskWriteHandler(USLOSS_Sysargs *args);

void phase4b_init(void) {
  systemCallVec[SYS_DISKSIZE] = diskSizeHandler;
  systemCallVec[SYS_DISKREAD] = diskReadHandler;
  systemCallVec[SYS_DISKWRITE] = diskWriteHandler;

  for (int i = 0; i < NUM_DISKS; i++) {
    diskMailbox[i] = MboxCreate(10, sizeof(int));
    diskQueue[i] = NULL;
  }
}

void phase4b_start_service_processes(void) {
  for (int i = 0; i < NUM_DISKS; i++) {
    spork("DiskDriver", DiskDriver, (void *)(long)i, USLOSS_MIN_STACK, 2);
  }
}

void phase4_start_service_processes(void) {
  spork("ClockDriver", ClockDriver, NULL, USLOSS_MIN_STACK, 1);
  for (int i = 0; i < 4; i++) {
    spork("TermDriver", TerminalDriver, (void *)(long)i, USLOSS_MIN_STACK, 1);
  }
  for (int i = 0; i < 4; i++) {
    int control = USLOSS_TERM_CTRL_RECV_INT(0) | USLOSS_TERM_CTRL_XMIT_INT(0);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, i, (void *)(long)control);
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
  phase4b_init();
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

void diskSizeHandler(USLOSS_Sysargs *args) {
  int unit = (int)(long)args->arg1;
  if (unit < 0 || unit >= NUM_DISKS) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  if (diskInfo[unit][2] == 0) {
    int status;
    USLOSS_DeviceRequest req;
    req.opr = USLOSS_DISK_TRACKS;
    req.reg1 = &diskInfo[unit][2];
    req.reg2 = NULL;
    USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &req);
    waitDevice(USLOSS_DISK_DEV, unit, &status);

    diskInfo[unit][0] = 512; // sector size (always 512 bytes)
    diskInfo[unit][1] = 16;  // sectors per track (always 16)
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
    args->arg4 = (void *)(long)-1;
    return;
  }

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

  if (!buffer || sectors <= 0 || unit < 0 || unit >= NUM_DISKS) {
    args->arg4 = (void *)(long)-1;
    return;
  }

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

void addDiskRequest(DiskRequest *req) {
  int unit = req->unit;

  if (diskQueue[unit] == NULL) {
    diskQueue[unit] = req;
  } else {
    DiskRequest *curr = diskQueue[unit];
    DiskRequest *prev = NULL;
    while (curr != NULL && curr->track <= req->track) {
      prev = curr;
      curr = curr->next;
    }
    if (prev == NULL) {
      req->next = diskQueue[unit];
      diskQueue[unit] = req;
    } else {
      req->next = curr;
      prev->next = req;
    }
  }

  int dummy = 0;
  MboxCondSend(diskMailbox[unit], &dummy, sizeof(int));
}

DiskRequest *getNextDiskRequest(int unit) {
  DiskRequest *req = diskQueue[unit];
  if (req)
    diskQueue[unit] = req->next;
  return req;
}

int DiskDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;

  // Send TRACKS command to get disk info
  USLOSS_DeviceRequest trackReq;
  int *tracks = &diskInfo[unit][2]; // Only getting number of tracks
  trackReq.opr = USLOSS_DISK_TRACKS;
  trackReq.reg1 = (void *)tracks;
  trackReq.reg2 = NULL;
  USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &trackReq);
  waitDevice(USLOSS_DISK_DEV, unit, &status);

  // Set defaults manually
  diskInfo[unit][0] = 512; // 512 bytes per block is hardcoded
  diskInfo[unit][1] = 16;  // 16 blocks per track is hardcoded

  while (1) {
    DiskRequest *req = getNextDiskRequest(unit);
    if (!req) {
      int dummy;
      MboxRecv(diskMailbox[unit], &dummy, sizeof(int));
      continue;
    }

    // Move head if needed
    USLOSS_DeviceRequest seek;
    seek.opr = USLOSS_DISK_SEEK;
    seek.reg1 = (void *)(long)(req->track);
    USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &seek);
    waitDevice(USLOSS_DISK_DEV, unit, &status);

    for (int i = 0; i < req->sectors; i++) {
      USLOSS_DeviceRequest op;
      op.opr = (req->opType == 0) ? USLOSS_DISK_READ : USLOSS_DISK_WRITE;
      op.reg1 = ((char *)req->buffer) + i * diskInfo[unit][0];
      op.reg2 = (void *)(long)(req->first + i);
      USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &op);
      waitDevice(USLOSS_DISK_DEV, unit, &status);
      if (status != USLOSS_DEV_READY) {
        req->status = status;
        unblockProc(req->pid);
        break;
      }
    }

    req->status = 0;
    unblockProc(req->pid);
  }

  return 0;
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
