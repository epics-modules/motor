eval 'exec perl -S $0 ${1+"$@"}'  # -*- Mode: perl -*-
    if $running_under_some_shell; # makeIocCdCommands.pl

use Cwd;

$cwd  = cwd();
#hack for sun4
$cwd =~ s|/tmp_mnt||;
$arch = $ARGV[0];

unlink("cdCommands");
open(OUT,">cdCommands") or die "$! opening cdCommands";
print OUT "startup = \"$cwd\"\n";

#appbin is kept for compatibility with 3.13.1 
$appbin = $cwd;
$appbin =~ s/iocBoot.*//;
$appbin = $appbin . "/bin/${arch}";
print OUT "appbin = \"$appbin\"\n";

$top = $cwd;
$top =~ s/\/iocBoot.*//;
print OUT "top = \"$top\"\n";
$topbin = "${top}/bin/${arch}";
#skip check that top/bin/${arch} exists; src may not have been builT
print OUT "topbin = \"$topbin\"\n";
# Add TOP to macro list.
$applications{TOP} = $top;

@files =();
push(@files,"$top/config/RELEASE");
foreach $file (@files) {
    if (-r "$file") {
	open(IN, "$file") or die "Cannot open $file\n";
	while ($line = <IN>) {
	    next if ( $line =~ /\s*#/ );
	    chomp($line);
	    $_ = $line;
	    ($prefix,$macro,$post) = /(.*)\s* \s*\$\((.*)\)(.*)/;
	    #test for "include" command
	    if ($prefix eq "include") {
		if ($macro eq "") {
		    # true if no macro is present
		    #the following looks for
		    #prefix = post
		    ($prefix,$post) = /(.*)\s* \s*(.*)/;
		}
		else {
		    $base = $applications{$macro};
		    if ($base eq "") {
			#print "error: $macro was not previously defined\n";
		    }
		    else {
			$post = $base . $post;
		    }
		}
		push(@files,"$post")
	    }
	    else {
		#the following looks for
		# prefix = $(macro)post
		($prefix,$macro,$post) = /(.*)\s*=\s*\$\((.*)\)(.*)/;
		if ($macro eq "") { # true if no macro is present
		    # the following looks for
		    # prefix = post
		    ($prefix,$post) = /(.*)\s*=\s*(.*)/;
		} else {
		    $base = $applications{$macro};
		    if ($base eq "") {
			#print "error: $macro was not previously defined\n";
		    } else {
			$post = $base . $post;
		    }
		}
		
		$prefix =~ s/^\s+|\s+$//g; # strip leading and trailing whitespace.
		$applications{$prefix} = $post;
		$app = lc($prefix);
		if ( -d "$post") { #check that directory exists
		    print OUT "$app = \"$post\"\n";
		}
		if ( -d "$post/bin/$arch") { #check that directory exists
		    print OUT "${app}bin = \"$post/bin/$arch\"\n";
		}
	    }
	}
	close IN;
    }
}
close OUT;
