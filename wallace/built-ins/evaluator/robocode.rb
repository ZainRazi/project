# encoding: utf-8
import('simple')
import('../chromosome/robocode')
import('../type/evaluator')
import('../fitness/simple')

define_type :robocode do
  extends(:evaluator)

  attribute :tank_evaluator,    ruby(:proc)
  attribute :threads,           ruby(:integer)
  attribute :path_robocode,     ruby(:string)

  template lambda { |base, opts = {}, &fun|
    base.path_robocode(opts[:path])
    base.threads(opts[:threads] || 1)
    base.tank_evaluator(fun)
  }

  composer do
    s = spec(:evaluator, :robocode).extend
    s.implement_method(:evaluate_tank, &tank_evaluator)
    s.constructor([], source: "super('#{path_robocode}', #{threads})")
    s
  end
  
end

define_specification :robocode do
  extends(:evaluator)

  ruby_file('tempfile')
  ruby_file('fileutils')
  ruby_file('pathname')

  attribute :path,    ruby(:string)
  attribute :threads, ruby(:integer)

  dependency s(:fitness, :simple)

  # Constructs a new robocode evaluator.
  constructor [
    p(:path,    ruby(:string)),
    p(:threads, ruby(:integer))
  ] do
    self.path = Pathname.new(path).cleanpath.to_s
    self.threads = threads

    # Construct the evolved robots sub-directory.
    FileUtils.mkdir_p("#{self.path}/robots/sample/evolved")
  end

  method :calculate_workload, returns: list(list(ruby(:integer))), accepts: [
    p(:size,    ruby(:integer)),
    p(:threads, ruby(:integer))
  ] do
    workload = []
    div = size / threads
    mod = size % threads
    start = 0
    threads.times do |i|
      length = div + (mod > 0 && mod > i ? 1 : 0)
      workload << [start, start + length]
      start += length
    end
    return workload
  end

  # Computes and returns the fitness of a single tank.
  abstract_method :evaluate_tank, returns: s(:fitness), accepts: [
    p(:rng,             s(:rng)),
    p(:population,      s(:population)),
    p(:statistics,      s(:statistics)),
    p(:representation,  s(:representation)),
    p(:chromosome,      s(:chromosome)),
    p(:tank,            s(:chromosome, :robocode))
  ]

  method :evaluate, returns: ruby(:integer), accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:population,  s(:population)),
    parameter(:termination, s(:termination))
  ] do

    # Evaluate the contents of each deme in series.
    population.demes.each do |deme|

      # Compute the list of individuals which require evaluation from the current
      # members of each deme, and their offspring. Avoid re-evaluating individuals
      # unless the problem being solved is dynamic.
      candidates = (deme.offspring + deme.contents).reject { |i| i.evaluated == true }

      # Calculate the number of individuals which will be evaluated.
      evals = candidates.length

      # If there is an evaluation limit on this algorithm, then further
      # restrict the list of candidates to the maximum number which may be
      # evaluated without breaching this limit. When restricting the number of
      # candidates, randomly chose which candidates will be evaluated and which
      # will not, to avoid any positional bias.

      # This code should be dynamically injected.
      if termination.criterion?('evaluations')
        remaining = [0, termination.evaluations.limit - statistics.evaluations].max
        if remaining < evals
          evals = remaining
          candidates = rng.sample(candidates, remaining)
        end
      end

      # Calculate how the candidate list should be split across threads.
      workload = calculate_workload(evals, threads)

      # Destroy the contents of the evolved tanks folder.
      FileUtils.rm_rf("#{path}/robots/sample/evolved/.", secure: true)

      # Construct and store tanks for each candidate.
      # Keep a tally of the number of viable tanks.
      viable = 0
      tanks = Array.new(evals)
      threads.times do |i|
        Thread.new(i) { |i|
          for i in workload[i][0] ... workload[i][1]
            begin
              tanks[i] = build_tank(candidates[i].species.representation.derive(candidates[i].genotype))
              viable += 1
            rescue
              tanks[i] = nil
            end
          end
        }.join
      end

      # Compile all of the tanks at once (this saves A LOT of time).
      # Only bother if we have some viable tanks to construct.
      if viable > 0
        system("javac -classpath #{path}/libs/robocode.jar #{path}/robots/sample/evolved/*.java")
      end

      # Evaluate the fitness of each of the tanks.
      threads.times do |i|
        Thread.new(i) { |i|
          for i in workload[i][0] ... workload[i][1]
            candidates[i].fitness = evaluate_tank(
              rng,
              population,
              statistics,
              candidates[i].species.representation,
              candidates[i].genotype,
              tanks[i]
            )
            candidates[i].evaluated = true
          end
        }.join
      end

      # Update the statistics to reflect the recent evaluations for this deme.
      statistics.evaluations += evals

    end

  end

  method :build_tank, returns: s(:chromosome, :robocode), accepts: [
    parameter(:body,  ruby(:string))
  ] do
    Chromosome::Robocode.new(self.path, body)
  end

  # Returns the results of a battle between -----
  method :battle, returns: ruby(:*), accepts: [
    parameter(:tanks,  list(ruby(:string))),
    parameter(:opts,        ruby(:*), {})
  ] do

      # Set the default options.
      opts[:rounds] ||= 10
      opts[:verbose] = false unless opts.key?(:verbose)

      # Compose the battle file to a temporary file.
      f = Tempfile.new(['battle-', '.battle'])
      f.write("#Battle Properties
        robocode.battleField.width=800
        robocode.battleField.height=600
        robocode.battle.numRounds=#{opts[:rounds]}
        robocode.battle.gunCoolingRate=0.1
        robocode.battle.rules.inactivityTime=450
        robocode.battle.hideEnemyNames=true
        robocode.battle.selectedRobots=#{tanks.join(',')}
      ")

      # Construct a temporary file to hold the results of the battle.
      f_results = Tempfile.new(['results-', '.txt'])

      # Close the file handler, allowing Robocode to access the battle file.
      f.close
      f_results.close

      # Compose the Robocode command line request.
      cmd = "java"

      # Set the robot path.
      cmd << " -DROBOTPATH=#{path}/robots/"

      # Specify the maximum heap size for the Java VM.
      cmd << " -Xmx512M"

      # Disable canonical caching of file names to prevent issues with Windows.
      cmd << " -Dsun.io.useCanonCaches=false"

      cmd << " -Djava.awt.headless=true"

      # Provide Java with the location of the Robocode class.
      cmd << " -cp #{path}/libs/robocode.jar robocode.Robocode"
      
      # Specify the location of the battle file.
      cmd << " -battle #{f.path}"

      # Disable visual output.
      cmd << " -nodisplay"

      # Disable sound.
      cmd << " -nosound"

      # Specify the location of the results file.
      cmd << " -results #{f_results.path}"




      # Silence the stdout.
      cmd << " >NUL" unless opts[:verbose]

      # Execute the command.
      system(cmd)

      # Parse the contents of the results file.
      results = {}
      File.open(f_results.path, "rb") do |f|
          f.read.lines.each_with_index do |row, i|
            row = row.strip.split("\s")
            results[row[1]] = {
              rank:           i + 1,
              score:          row[2].to_i,
              score_share:    row[3][1 ... -1].to_i,
              survival:       row[4].to_i,
              bullet_damage:  row[5].to_i,
              bullet_bonus:   row[6].to_i,
              ram_damage:     row[7].to_i,
              ram_bonus:      row[8].to_i,
              first_places:   row[9].to_i,
              second_places:  row[10].to_i,
              third_places:   row[11].to_i
          }
        end
      end

      # Destroy the temporary battle file.
      f_results.unlink
      f.unlink

      # Return the parsed results of the battle.
      return results

  end

end
