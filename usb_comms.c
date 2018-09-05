#include "usb_comms.h"
#include <usb/device.h>
#include <kernel/thread.h>
#include <kernel/io.h>
#include <kernel/dispatcher.h>
#include <stdio.h>
#include <string.h>
#include <kernel/ascii.h>
#include <meta/parse.h>

#define STACK_SIZE 1280
static thread_t _thread;
static char _stack[STACK_SIZE];
static void _service(void *args);
static mutex_t _write_mutex;
static io_entry_t _io;
static unsigned _view_index = 0;

void usb_comms_initialize()
{
	usb_device_initialize();
	mutex_create(&_write_mutex);

	event_t start;
	event_create(&start, EVENTF_NONE);
	thread_create(&_thread, -1, _stack, STACK_SIZE, _service, &start);
	event_wait(&start, TIMEOUT_NEVER);
}

void usb_comms_write(unsigned view, const char *format, ...)
{
	mutex_lock(&_write_mutex);
	if (_view_index == view)
	{
		char buf[256];
		va_list args;
		va_start(args, format);
		int length = vsprintf(buf, format, args);
		io_write(&_io, buf, length);
	}
	mutex_unlock(&_write_mutex);
}

static int _config(char *buf)
{
	//TODO
	return 0;
}

static int _write(io_entry_t *io, const char *format, ...)
{
	mutex_lock(&_write_mutex);
	char buf[256];
	va_list args;
	va_start(args, format);
	int length = vsprintf(buf, format, args);
	int done = io_write(io, buf, length);
	mutex_unlock(&_write_mutex);
	return done;
}

static int _read(io_entry_t *io, char *buf, unsigned length)
{
	int done = 0;
	while(done < length)
	{
		char c;
		int done2 = io_read(io, &c, 1);
		if (done2 < 0)
			return done2;
		
		if (done2 != 0)
		{
			if (c >= 32)
			{
				buf[done++] = c;
			}
			else if (c == 13)
			{
				break;
			}
		}
	}
	return done;
}

static bool _cmp(const char **pbuf, const char *name)
{
	const char *buf = *pbuf;
	while(*buf == ' ') buf++;

	int done = 0;
	char c;
	while(c = name[done], c != '\0')
	{
		if (buf[done] == c)
		{
			done++;
			continue;
		}
		return false;
	}
	
	c = buf[done];
	if (c == '\0')
	{
		*pbuf = buf + done;
		return true;
	}
	if (c == ' ')
	{
		do { done++; }
		while(c = buf[done], c == ' ');
		*pbuf = buf + done;
		return true;
	}
	return false;
}

static void _view(io_entry_t *io, unsigned index)
{
	if (index != 0)
	{
		static char dummy[16];
		_view_index = index;
		_read(io, dummy, sizeof(dummy));	// wait return
		_view_index = 0;
	}
}

static void _parse(io_entry_t *io, const char *cmd)
{
	meta_var_t *var;
	if (_cmp(&cmd, "set") || _cmp(&cmd, "s"))
	{
		var = meta_parse_var_name(&cmd);
		if (var != nullptr)
		{
			if (var->Type & META_ARRAY)
			{
				unsigned index;
				if (meta_parse_index(&cmd, &index))
				{
					if (meta_var_parse_and_set(var, index, &cmd))
					{
						_write(io, "var %s[%d] = %m\n", var->Name, index, var); 
					}
					else _write(io, "set %s[%d]: value?\n", var->Name, index);
				}
				else _write(io, "set %s: index?\n", var->Name);
			}
			else
			{
				if (meta_var_parse_and_set(var, 0, &cmd))
				{
					_write(io, "%s = %m\n", var->Name, var); 
				}
				else _write(io, "set %s: value?\n", var->Name);
			}
		}
		else _write(io, "set: var not found\n");
	}
	else if (_cmp(&cmd, "list") || _cmp(&cmd, "l"))
	{
		unsigned index = 0;
		while(var = meta_get_var(index++), var != nullptr)
		{
			_write(io, "%s: %m\n", var->Name, var);
		}
	}
	else if (_cmp(&cmd, "view") || _cmp(&cmd, "v"))
	{
		unsigned view;
		if (meta_parse_index(&cmd, &view))
		{
			_view(io, view);
		}
		else _write(io, "view: index?\n");
	}
	else if (_cmp(&cmd, "save"))
	{
		bool save = false; //persist_save();
		_write(io, "persist save: %s\n", save ? "ok" : "failed");
	}
	else if (strlen(cmd) != 0) 
		_write(io, "unknown command: %s\n", cmd);
}

static void _prompt(char *buf)
{
	unsigned int vbat, curr1, curr2;
	meta_var_get_by_name("vbat", 0, 1, &vbat);
	meta_var_get_by_name("curr1", 0, 1, &curr1);
	meta_var_get_by_name("curr2", 0, 1, &curr2);
	sprintf(buf, "%d mV %d A(1), %d A(2) $", vbat, curr1, curr2);
}

static void _service(void *args)
{
	event_t *start = (event_t *)args;
	event_set(start);

	char buf[256];
	while(1)
	{
		io_error_t err = io_open(&_io, "/dev/ftdi0", IOF_NONE);
		if (err == IO_OK)
		{
			io_set_timeout(&_io, 1000);

			int done = _write(&_io, "debug prompt built %s.\n", __DATE__);
			if (done < 0) continue;

			while(1)
			{
				_prompt(buf);
				done = _write(&_io, "%s \x1b[12l", buf);
				if (done < 0) break;

				done = _read(&_io, buf, sizeof(buf));
				if (done < 0) break;
				_write(&_io, "\x1b[12h\n");

				buf[done] = '\0';
				_parse(&_io, buf);
			}

			io_close(&_io);
		}
		else thread_sleep(500);
	}
}


