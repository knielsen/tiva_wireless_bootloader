#! /usr/bin/perl

use strict;
use warnings;

use Time::HiRes;
use IO::Stty;

my $tty;
my $binfile;

if (@ARGV == 3 && $ARGV[0] eq '-t') {
  $tty = $ARGV[1];
  $binfile = $ARGV[2];
} elsif (@ARGV == 1) {
  $tty = '/dev/ttyACM0';
  $binfile = $ARGV[0];
} else {
  print STDERR "Usage: $0 [-f /dev/ttyACMx] file-to-flash.bin\n";
  exit 1;
}

open BINFILE, "<", $binfile
    or die "Could not open file '$binfile': $!\n";
binmode BINFILE;


open TTY, "+<", $tty
    or die "Failed to open programmer on $tty: $!\n";

select TTY;
$| = 1;
select STDOUT;
$| = 1;
binmode TTY;
IO::Stty::stty(\*TTY, 'raw');


sub mk_packet {
  my $packet = '';
  while (@_) {
    $packet .= chr(shift);
  }
  while (length($packet) < 32) {
    $packet .= chr(0);
  }
  return $packet;
}


# Drain any lingering output from earlier runs.
sub drain_usb_resp {
  for (;;) {
    my $resp = '';
    my $rin = '';
    vec($rin, fileno(TTY), 1) = 1;
    if (select($rin, undef, undef, 0.1))
    {
      my $res= sysread(TTY, $resp, 1024);
      next if $res;
    }
    last;
  }
}


sub read_usb_resp {
  my $resp = '';

  for (;;) {
    my $timeout = 10;
    my $rin = '';
    vec($rin, fileno(TTY), 1) = 1;
    if (!select($rin, undef, undef, $timeout))
    {
      $resp =~ s/\r/\\r/gs;
      $resp =~ s/\n/\\n/gs;
      die "No response from programmer in $timeout seconds, abort. ".
          "Partial receive: '$resp'\n";
    }
    my $res= sysread(TTY, $resp, 1, length($resp));
    die "Got error on read of TTY: $!\n" if !defined $res;
    die "EOF on tty\n" unless $res;
    die "What?!? sysread() returns unexpected value $res on read of 1 byte from tty"
        if $res != 1;
    if (substr($resp, -1) eq "\n") {
      if (substr($resp, 0, 1) eq '!' || substr($resp, 0, 11) eq 'fifo status') {
        $resp =~ s/\r/\\r/gs;
        $resp =~ s/\n/\\n/gs;
        print STDERR "DBG: '$resp'\n";
        $resp = '';
      } else {
        chomp($resp);
        chop($resp) if substr($resp, -1) eq "\r";
        return $resp;
      }
    }
  }
}


sub send_packet {
  my ($packet, $wait_response) = @_;

  #print STDERR "Sending packet: ", ord(substr($packet, 0, 1)), " ", ord(substr($packet, 1, 1)), "...\n";
  print TTY $packet;
  if ($wait_response) {
    my $resp = read_usb_resp();
    if ($resp =~ /^E: ([^\r]*)$/) {
      die "Got error response from programmer: $1\n";
    } elsif ($resp !~ /^OK\r?$/) {
      die "Got unexpected response from programmer: $resp\n";
    }
  }
}


print "Contacting bootloader..";
my $tries = 0;
for (;;) {
  drain_usb_resp();
  eval {
    # First send cmd to make lm4f_pov reset into bootloader
    send_packet(mk_packet(254, 255), 0);
    # Now send hello packet.
    send_packet(mk_packet(254, 254), 1);
    1;
  } and last;
  print STDERR ".";
  # Exit debug mode to send the 254/255 reset packet in non-ack mode correctly.
  send_packet(mk_packet(254, 251), 0);
  Time::HiRes::sleep(0.4);
  ++$tries;
  die "No response from bootloader, giving up\n"
      if $tries > 500;
}
print STDERR " done!\n";

my $blocknum = 4;
for (;;)
{
  my $block = "\xff" x 1024;
  my $res = sysread(BINFILE, $block, 1024);
  die "Error reading file '$binfile': $!\n"
      unless defined($res);
  last unless $res > 0;

  my $idx = 0;
  print "Send block $blocknum... ";
  while ($idx <= 34 && ($idx*30) < length($block)) {
    my $packet = chr(254) . chr ($idx) . substr($block, $idx*30, 30);
    $packet .= ("\xff" x (32-length($packet))) if length($packet) < 32;
    send_packet($packet, 1);
    ++$idx;
  }
  send_packet(mk_packet(254, 252, $blocknum, 0, 0, 0), 1);
  print "flashed\n";
  ++$blocknum;
  last unless $res == 1024;
}

print "\nFlashing completed, reset target.\n";
# Send a request to reset the target.
send_packet(mk_packet(254, 253), 0);
# Send a debug exit command. pov_sender goes back to transmitting animation frames.
send_packet(mk_packet(254, 251), 0);
