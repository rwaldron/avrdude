// ----------------------------------------------------------------------
// Driver for the "usbtiny" programmer.
//
// Copyright (C) 2006 Dick Streefland
//
// This is free software, licensed under the terms of the GNU General
// Public License as published by the Free Software Foundation.
// ----------------------------------------------------------------------

#include "ac_cfg.h"

#include <string.h>
#include "pgm.h"
#include "avr.h"

extern	char*		progname;

#if defined(HAVE_LIBUSB)      // we use LIBUSB to talk to the board

#include <usb.h>


// these are specifically assigned to USBtiny courtesy of Adafruit Industries,
// if you need your own VID and PIDs you can get them for cheap from
// www.mecanique.co.uk so please don't reuse these. Thanks!

#define USBDEV_VENDOR 0x1781
#define USBDEV_PRODUCT 0x0C9F


enum
{
	// Generic requests
	USBTINY_ECHO,		// echo test
	USBTINY_READ,		// read byte (wIndex:address)
	USBTINY_WRITE,		// write byte (wIndex:address, wValue:value)
	USBTINY_CLR,		// clear bit (wIndex:address, wValue:bitno)
	USBTINY_SET,		// set bit (wIndex:address, wValue:bitno)
	// Programming requests
	USBTINY_POWERUP,	// apply power (wValue:SCK-period, wIndex:RESET)
	USBTINY_POWERDOWN,	// remove power from chip
	USBTINY_SPI,		// issue SPI command (wValue:c1c0, wIndex:c3c2)
	USBTINY_POLL_BYTES,	// set poll bytes for write (wValue:p1p2)
	USBTINY_FLASH_READ,	// read flash (wIndex:address)
	USBTINY_FLASH_WRITE,	// write flash (wIndex:address, wValue:timeout)
	USBTINY_EEPROM_READ,	// read eeprom (wIndex:address)
	USBTINY_EEPROM_WRITE,	// write eeprom (wIndex:address, wValue:timeout)
};

enum
{
	RESET_LOW	= 0,
	RESET_HIGH	= 1,
	SCK_MIN		= 1,	// usec (target clock >= 4 MHz)
	SCK_MAX		= 250,	// usec (target clock >= 16 KHz)
	SCK_DEFAULT	= 10,	// usec (target clock >= 0.4 MHz)
	CHUNK_SIZE	= 128,	// must be power of 2 less than 256
	USB_TIMEOUT	= 500,	// msec
};

typedef	unsigned char	byte_t;
typedef	unsigned int	uint_t;
typedef	unsigned long	ulong_t;

extern	int		avr_write_byte_default ( PROGRAMMER* pgm, AVRPART* p,
			                         AVRMEM* mem, ulong_t addr,
						 byte_t data );
extern	int		verbose;

static	usb_dev_handle*	usb_handle;
static	int		sck_period;
static	int		chunk_size;
static	AVRMEM*		cache_mem;
static	ulong_t		cache_base;
static	byte_t		cache_buf[CHUNK_SIZE];
static	int		cache_disable;


// ----------------------------------------------------------------------

static	void	usb_control ( int req, int val, int index )
{
	usb_control_msg( usb_handle,
			 USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			 req, val, index, NULL, 0, USB_TIMEOUT );
}

static	int	usb_in ( int req, int val, int index, byte_t* buf, int buflen, int umax )
{
	int	n;
	int	timeout;

	timeout = USB_TIMEOUT + (buflen * umax) / 1000;
	n = usb_control_msg( usb_handle,
			USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			req, val, index, (char*) buf, buflen, timeout );
	if	( n != buflen )
	{
		fprintf( stderr, "USB read error: expected %d, got %d\n", buflen, n );
		return -1;
	}
	return 0;
}

static	int	usb_out ( int req, int val, int index, byte_t* buf, int buflen, int umax )
{
	int	n;
	int	timeout;

	timeout = USB_TIMEOUT + (buflen * umax) / 1000;
	n = usb_control_msg( usb_handle,
			USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			req, val, index, (char*) buf, buflen, timeout );
	if	( n != buflen )
	{
		fprintf( stderr, "USB write error: expected %d, got %d\n", buflen, n );
		return -1;
	}
	return 0;
}

static	int	usbtiny_avr_op ( PROGRAMMER* pgm, AVRPART* p, int op, byte_t res[4] )
{
	byte_t	cmd[4];

	if	( ! p->op[op] )
	{
		fprintf( stderr, "Operation %d not defined\n", op );
		return -1;
	}
	memset( cmd, 0, sizeof(cmd) );
	avr_set_bits( p->op[op], cmd );
	return pgm->cmd( pgm, cmd, res );
}

// ----------------------------------------------------------------------

static	int	usbtiny_open ( PROGRAMMER* pgm, char* name )
{
	struct	usb_bus*	bus;
	struct	usb_device*	dev = 0;

	usb_init();
	usb_find_busses();
	usb_find_devices();
	for	( bus = usb_busses; bus; bus = bus->next )
	{
		for	( dev = bus->devices; dev; dev = dev->next )
		{
			if	(  dev->descriptor.idVendor == USBDEV_VENDOR
				&& dev->descriptor.idProduct == USBDEV_PRODUCT
				)
			{
				usb_handle = usb_open( dev );
				if	( ! usb_handle )
				{
					fprintf( stderr,
						"Cannot open USB device: %s\n",
						usb_strerror() );
					return -1;
				}
				return 0;
			}
		}
	}
	fprintf( stderr, "Could not find USB device 0x%x/0x%x\n",
		USBDEV_VENDOR, USBDEV_PRODUCT );
	return -1;
}

static	void	usbtiny_close ( PROGRAMMER* pgm )
{
	if	( ! usb_handle )
	{
		return;
	}
	usb_close( usb_handle );
	usb_handle = NULL;
}

static	void	usbtiny_set_chunk_size ( int period )
{
	chunk_size = CHUNK_SIZE;
	while	( chunk_size > 8 && period > 16 )
	{	// Reduce the chunk size for a slow SCK to reduce
		// the maximum time of a single USB transfer.
		chunk_size >>= 1;
		period >>= 1;
	}
}

static	int	usbtiny_set_sck_period ( PROGRAMMER* pgm, double v )
{
	sck_period = (int) (v * 1e6 + 0.5);
	if	( sck_period < SCK_MIN )
	{
		sck_period = SCK_MIN;
	}
	if	( sck_period > SCK_MAX )
	{
		sck_period = SCK_MAX;
	}
	printf( "%s: Setting SCK period to %d usec\n", progname, sck_period );
	usb_control( USBTINY_POWERUP, sck_period, RESET_LOW );
	usbtiny_set_chunk_size( sck_period );
	return 0;
}

static	int	usbtiny_initialize ( PROGRAMMER* pgm, AVRPART* p )
{
	byte_t	res[4];

	if	( pgm->bitclock > 0.0 )
	{	// -B option specified: convert to valid range for sck_period
		usbtiny_set_sck_period( pgm, pgm->bitclock );
	}
	else
	{	// -B option not specified: use default
		sck_period = SCK_DEFAULT;
		if	( verbose )
		{
			printf( "%s: Using SCK period of %d usec\n",
				progname, sck_period );
		}
		usb_control( USBTINY_POWERUP, sck_period, RESET_LOW );
		usbtiny_set_chunk_size( sck_period );
	}
	usleep( 50000 );
	if	( ! usbtiny_avr_op( pgm, p, AVR_OP_PGM_ENABLE, res ) )
	{	// no response, RESET and try again
		usb_control( USBTINY_POWERUP, sck_period, RESET_HIGH );
		usb_control( USBTINY_POWERUP, sck_period, RESET_LOW );
		usleep( 20000 );
		if	( ! usbtiny_avr_op( pgm, p, AVR_OP_PGM_ENABLE, res ) )
		{	// give up
			return -1;
		}
	}
	return 0;
}

static	void	usbtiny_powerdown ( PROGRAMMER* pgm )
{
	if	( ! usb_handle )
	{
		return;
	}
	usb_control( USBTINY_POWERDOWN, 0, 0 );
}

static	int	usbtiny_cmd ( PROGRAMMER* pgm, byte_t cmd[4], byte_t res[4] )
{
	int	r;

	cache_mem = NULL;
	memset( res, '\0', sizeof(res) );
	r = usb_in( USBTINY_SPI, (cmd[1] << 8) | cmd[0], (cmd[3] << 8) | cmd[2],
	            (char*) res, sizeof(res), 8 * sck_period );
	if	( verbose > 1 )
	{
		printf( "CMD: [%02x %02x %02x %02x] [%02x %02x %02x %02x]\n",
		        cmd[0], cmd[1], cmd[2], cmd[3],
			res[0], res[1], res[2], res[3] );
	}
	return (r == 0 && res[2] == cmd[1]);
}

static	int	usbtiny_chip_erase ( PROGRAMMER* pgm, AVRPART* p )
{
	byte_t	res[4];
	int	r;

	r = usbtiny_avr_op( pgm, p, AVR_OP_CHIP_ERASE, res );
	usleep( p->chip_erase_delay );
	usb_control( USBTINY_POWERUP, sck_period, RESET_HIGH );
	pgm->initialize( pgm, p );
	return r;
}

static	void	usbtiny_display ( PROGRAMMER* pgm, char* p )
{
}

static	void	usbtiny_enable ( PROGRAMMER* pgm )
{
}

static	void	usbtiny_disable ( PROGRAMMER* pgm )
{
}

static	int	usbtiny_paged_load ( PROGRAMMER* pgm, AVRPART* p, AVRMEM* m,
		                     int page_size, int n_bytes )
{
	int	i;
	int	chunk;
	int	req;

	req = ( strcmp( m->desc, "flash" ) == 0
	      ? USBTINY_FLASH_READ
	      : USBTINY_EEPROM_READ
	      );
	for	( i = 0; i < n_bytes; i += chunk )
	{
		chunk = chunk_size;
		if	( chunk > n_bytes - i )
		{
			chunk = n_bytes - i;
		}
		usb_in( req, 0, i, m->buf + i, chunk, 32 * sck_period );
		report_progress( i + chunk, n_bytes, NULL );
	}
	return n_bytes;
}

static	int	usbtiny_paged_write ( PROGRAMMER* pgm, AVRPART* p, AVRMEM* m,
		                      int page_size, int n_bytes )
{
	int	i;
	int	chunk;
	int	next;
	int	req;
	int	delay;

	req = ( strcmp( m->desc, "flash" ) == 0
	      ? USBTINY_FLASH_WRITE
	      : USBTINY_EEPROM_WRITE
	      );
	delay = 0;
	if	( ! m->paged )
	{
		i = (m->readback[1] << 8) | m->readback[0];
		usb_control( USBTINY_POLL_BYTES, i, 0 );
		delay = m->max_write_delay;
	}
	for	( i = 0; i < n_bytes; i = next )
	{
		chunk = chunk_size;
		if	( m->paged && chunk > page_size )
		{
			chunk = page_size;
		}
		if	( chunk > n_bytes - i )
		{
			chunk = n_bytes - i;
		}
		usb_out( req, delay, i, m->buf + i, chunk, 32 * sck_period + delay );
		next = i + chunk;
		if	(  m->paged
			&& ((next % page_size) == 0 || next == n_bytes)
			)
		{
			avr_write_page( pgm, p, m, (unsigned long) i );
		}
		report_progress( next, n_bytes, NULL );
	}
	return n_bytes;
}

static	int	usbtiny_read_byte ( PROGRAMMER* pgm, AVRPART* p, AVRMEM* m,
		                    ulong_t addr, byte_t* value )
{
	ulong_t	base;
	int	req;
	int	size;

	if	( cache_disable > 0 )
	{
	  return avr_read_byte_default(pgm, p, m, addr, value);
				       // fall back to avr_read_byte_default()
	}
	if	(  strcmp( m->desc, "flash" ) != 0
		&& strcmp( m->desc, "eeprom" ) != 0
		)
	{
	  return avr_read_byte_default(pgm, p, m, addr, value);
				       // fall back to avr_read_byte_default()
	}
	base = addr & ~ (chunk_size - 1);
	if	( cache_mem != m || cache_base != base )
	{
		req = ( strcmp( m->desc, "flash" ) == 0
		      ? USBTINY_FLASH_READ
		      : USBTINY_EEPROM_READ
		      );
		size = (m->size < chunk_size ? m->size : chunk_size);
		if	( usb_in( req, 0, (int) base, cache_buf, size, 32 * sck_period ) != 0 )
		{
			cache_mem = NULL;
			return -1;
		}
		cache_mem = m;
		cache_base = base;
	}
	*value = cache_buf[addr - base];
	return 0;
}

static	int	usbtiny_write_byte ( PROGRAMMER* pgm, AVRPART* p, AVRMEM* m,
		                     ulong_t addr, byte_t value )
{
	int	r;

	cache_disable++;
	r = avr_write_byte_default( pgm, p, m, addr, value );
	cache_disable--;
	return r;
}

extern	void	usbtiny_initpgm ( PROGRAMMER* pgm )
{
	strcpy( pgm->type, "USBTINY" );
	pgm->initialize		= usbtiny_initialize;
	pgm->display		= usbtiny_display;
	pgm->enable		= usbtiny_enable;
	pgm->disable		= usbtiny_disable;
	pgm->powerup		= NULL;
	pgm->powerdown		= usbtiny_powerdown;
	pgm->program_enable	= NULL;
	pgm->chip_erase		= usbtiny_chip_erase;
	pgm->cmd		= usbtiny_cmd;
	pgm->open		= usbtiny_open;
	pgm->close		= usbtiny_close;
	pgm->paged_load		= usbtiny_paged_load;
	pgm->paged_write	= usbtiny_paged_write;
	pgm->read_byte		= usbtiny_read_byte;
	pgm->write_byte		= usbtiny_write_byte;
	pgm->set_sck_period	= usbtiny_set_sck_period;
}

#else  /* !HAVE_LIBUSB */

void usbtiny_initpgm(PROGRAMMER * pgm)
{
  fprintf(stderr,
	  "%s: libusb access not available in this configuration\n",
	  progname);
}

#endif /* HAVE_LIBUSB */
